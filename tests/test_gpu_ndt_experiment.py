"""Offline experiment checks. CUDA tests use explicit C++ doubles, not a GPU."""
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.gpu_ndt import analyze, dataset, prepare, run, runtime
from ndt_realtime_test_support import method


@pytest.fixture
def base(tmp_path):
    base = tmp_path/'base'
    for name in ('CMakeLists.txt','package.xml','msg/ScanMatchingStatus.msg',
                 'include/hdl_localization/delta_estimater.hpp',
                 'include/kkl/alg/unscented_kalman_filter.hpp'):
        p = base/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('ament_auto_package()\n' if name=='CMakeLists.txt' else 'base snapshot\n')
    return base


def test_experimental_overlay_leaves_inputs_untouched(base, tmp_path):
    before = prepare.inventory(base)
    result = prepare.prepare(base, tmp_path/'experiment')
    assert prepare.inventory(base) == before
    assert result['default_backend'] == 'NDT_OMP'
    assert result['production_activated'] is False
    assert result['cuda_enabled_by_default'] is False
    runtime.verify_sources(tmp_path/'experiment')


@pytest.mark.parametrize('signature', [
    'void LidarCallback(', 'void ProcessPendingScan(', 'void QueueScan(',
    'void PublishOdometry(', 'void PublishScanMatchingStatus(',
    'bool FuseNdtCorrectionWithLegOdom(', 'bool TryPromoteStableNdtCandidate(',
    'std::vector<float> TargetDistances(',
])
def test_all_quality_fusion_scan_and_output_paths_are_exactly_preserved(signature):
    original = (ROOT/'vendor_patch/hdl_localization_app.hpp').read_text()
    experimental = prepare.experiment_header(original)
    assert method(original, signature) == method(experimental, signature)


def test_current_ukf_and_pose_estimator_overlaid_without_changes(base, tmp_path):
    out = tmp_path/'experiment'
    prepare.prepare(base, out)
    for rel in ['src/hdl_localization/pose_estimator.cpp',
                'include/hdl_localization/pose_estimator.hpp',
                'include/hdl_localization/quaternion_state.hpp']:
        assert (out/'src/hdl_localization'/rel).read_bytes() == (ROOT/'vendor_patch/hdl_localization'/rel).read_bytes()


def test_gpu_preflight_before_sensor_wait_and_only_one_gpu_matcher():
    text = prepare.experiment_header((ROOT/'vendor_patch/hdl_localization_app.hpp').read_text())
    assert text.index('preflight(reg_method)') < text.index('CalLidar2BaseStaticTF();')
    assert 'new DeltaEstimater(\n            lite3_gpu_ndt::make_registration<PointT>(\n                "NDT_OMP"' in text
    assert 'boost::shared_ptr<fast_gicp::NDTCuda' not in text


def test_launch_copies_localization_settings_and_omits_navigation():
    original = (ROOT/'patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py').read_text()
    text = prepare.experiment_launch(original)
    a, b = ast.parse(original), ast.parse(text)
    def params(tree):
        call = next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign)
                    and any(isinstance(t,ast.Name) and t.id=='localization' for t in n.targets))
        return next(k.value.elts[0] for k in call.keywords if k.arg=='parameters')
    before, after = params(a), params(b)
    for k, v, k2, v2 in zip(before.keys,before.values,after.keys,after.values):
        assert ast.dump(k) == ast.dump(k2)
        if k.value != 'reg_method':
            assert ast.dump(v) == ast.dump(v2)
    assert 'default_value="NDT_OMP"' in text
    assert 'ROS_DOMAIN_ID' in text and 'ROS_LOCALHOST_ONLY' in text
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name)
                   and n.func.id=='IncludeLaunchDescription' for n in ast.walk(b))


@pytest.mark.parametrize('case', ['existing','nested','missing','symlink'])
def test_workspace_overwrite_and_incomplete_input_refused(base,tmp_path,case):
    target = tmp_path/'out'
    if case=='existing':target.mkdir()
    if case=='nested':target = base/'out'
    if case=='missing':(base/'package.xml').unlink()
    if case=='symlink':(base/'linked').symlink_to(base/'package.xml')
    with pytest.raises(ValueError):prepare.prepare(base,target)


def test_dependency_arch_patch_preserves_external_flags_and_targets_sm72():
    text = prepare.patch_cuda_arch('set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")')
    assert '${CUDA_NVCC_FLAGS}' in text
    assert 'arch=compute_72,code=sm_72' in text
    assert 'arch=compute_72,code=compute_72' in text
    with pytest.raises(ValueError):prepare.patch_cuda_arch('unknown upstream layout')


def test_cuda_configuration_fails_explicitly_on_vm_without_nvcc(tmp_path):
    if shutil.which('nvcc'):
        pytest.skip('this check is specific to a host without CUDA')
    r = subprocess.run(['cmake','-S',str(ROOT/'experiments/gpu_ndt'),'-B',str(tmp_path/'build'),
                        '-DLITE3_GPU_NDT_ENABLED=ON'],capture_output=True,text=True)
    assert r.returncode != 0
    assert 'CUDA' in r.stdout+r.stderr


FAKE = r'''
#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <string>
#include <limits>
struct State {
 int devices=1,init_error=0,sync_error=0,submission_error=0,cuda_calls=0;
 int gpu_instances=0,source_uploads=0,target_uploads=0,map_builds=0,source_builds=0;
 int mode=-1,search=-1;double resolution=0,epsilon=0;
 bool converged=true,bad_pose=false,bad_output=false;
}state;
namespace pcl {
struct PointXYZI {float x=1,y=2,z=3,intensity=0;};
template<class P>struct PointCloud:std::vector<P> {
 using Ptr=PTR<PointCloud>;using ConstPtr=PTR<const PointCloud>;
};
template<class P,class T=P>class Registration {
public:
 using Ptr=PTR<Registration>;using Matrix4=Eigen::Matrix4f;
 using PointCloudSource=PointCloud<P>;using PointCloudSourceConstPtr=typename PointCloudSource::ConstPtr;
 using PointCloudTargetConstPtr=typename PointCloud<T>::ConstPtr;
 virtual ~Registration()=default;
 virtual void setInputSource(const PointCloudSourceConstPtr& c){input_=c;}
 virtual void setInputTarget(const PointCloudTargetConstPtr& c){target_=c;}
 void align(PointCloudSource& o,const Matrix4& g=Matrix4::Identity()){computeTransformation(o,g);}
 bool hasConverged()const{return converged_;}
 Matrix4 getFinalTransformation()const{return final_transformation_;}
protected:
 PointCloudSourceConstPtr input_;PointCloudTargetConstPtr target_;
 bool converged_=false;Matrix4 final_transformation_=Matrix4::Identity();
 virtual void computeTransformation(PointCloudSource& o,const Matrix4& g){
  o=*input_;converged_=state.converged;final_transformation_=g;
  if(state.bad_pose)final_transformation_(0,0)=std::numeric_limits<float>::quiet_NaN();
  if(state.bad_output)o[0].x=std::numeric_limits<float>::quiet_NaN();
 }
};
}
namespace pclomp {
enum Search{DIRECT1=1,DIRECT7=7};
template<class P,class T>struct NormalDistributionsTransform:pcl::Registration<P,T>{
 using Ptr=PTR<NormalDistributionsTransform>;
 void setTransformationEpsilon(double v){state.epsilon=v;}
 void setResolution(double v){state.resolution=v;}
 void setNeighborhoodSearchMethod(Search v){state.search=int(v);}
};
}
namespace fast_gicp {
enum class NDTDistanceMode{P2D=1,D2D=2};enum class NeighborSearchMethod{DIRECT1=1,DIRECT7=7};
namespace cuda {struct NDTCudaCore {
 bool target=false,source=false;
 void create_target_voxelmap(){if(!target){++state.map_builds;target=true;}}
 void create_voxelmaps(){create_target_voxelmap();if(state.mode==2&&!source){++state.source_builds;source=true;}}
};}
template<class P,class T>struct LsqRegistration:pcl::Registration<P,T>{};
template<class P,class T>struct NDTCuda:LsqRegistration<P,T>{
 using Parent=pcl::Registration<P,T>;
 using typename Parent::PointCloudSource;using typename Parent::Matrix4;
 using typename Parent::PointCloudSourceConstPtr;using typename Parent::PointCloudTargetConstPtr;
 std::unique_ptr<cuda::NDTCudaCore>ndt_cuda_{new cuda::NDTCudaCore};
 NDTCuda(){++state.gpu_instances;}
 void setResolution(double v){state.resolution=v;}
 void setDistanceMode(NDTDistanceMode v){state.mode=int(v);}
 void setNeighborSearchMethod(NeighborSearchMethod v){state.search=int(v);}
 void setInputSource(const PointCloudSourceConstPtr& c)override{
  if(c==this->input_)return;Parent::setInputSource(c);++state.source_uploads;ndt_cuda_->source=false;
 }
 void setInputTarget(const PointCloudTargetConstPtr& c)override{
  if(c==this->target_)return;Parent::setInputTarget(c);++state.target_uploads;ndt_cuda_->target=false;
 }
};
}
using cudaError_t=int;constexpr cudaError_t cudaSuccess=0;
inline const char* cudaGetErrorString(int){return "mock CUDA error";}
inline int cudaGetDeviceCount(int* n){++state.cuda_calls;*n=state.devices;return 0;}
inline int cudaFree(void*){++state.cuda_calls;return state.init_error;}
inline int cudaGetLastError(){++state.cuda_calls;return state.submission_error;}
inline int cudaDeviceSynchronize(){++state.cuda_calls;return state.sync_error;}
'''
MAIN = r'''
#include <lite3_gpu_ndt/backend.hpp>
#include <cassert>
#include <iostream>
using P=pcl::PointXYZI;using C=pcl::PointCloud<P>;
template<class F>void throws(F f){bool thrown=false;try{f();}catch(const std::exception&){thrown=true;}assert(thrown);}
int main(int argc,char** argv){
 assert(argc==2);std::string mode=argv[1];
 auto profile=std::make_shared<lite3_gpu_ndt::Profile>();
 if(mode=="cpu"){
  state.devices=0;lite3_gpu_ndt::preflight("NDT_OMP");
  auto n=lite3_gpu_ndt::make_registration<P>("NDT_OMP",1,"DIRECT1",profile);
  assert(n&&state.gpu_instances==0&&state.cuda_calls==0);
  assert(state.epsilon==.01&&state.resolution==1&&state.search==1);return 0;
 }
 if(mode=="invalid"){
  for(auto s:{"NDT_CUDA","NDT_CUDA_D2D_BAD","unknown"})
   throws([&]{lite3_gpu_ndt::make_registration<P>(s,1,"DIRECT1");});
  throws([&]{lite3_gpu_ndt::make_registration<P>("NDT_OMP",0,"DIRECT1");});
  throws([&]{lite3_gpu_ndt::make_registration<P>("NDT_OMP",1,"KDTREE");});
  assert(state.cuda_calls==0);return 0;
 }
#ifndef LITE3_GPU_NDT_ENABLED
 throws([&]{lite3_gpu_ndt::make_registration<P>("NDT_CUDA_P2D",1,"DIRECT1");});
 assert(state.gpu_instances==0);return 0;
#else
 if(mode=="no_device")state.devices=0;
 if(mode=="init_failure")state.init_error=1;
 if(mode=="sync_failure")state.sync_error=1;
 if(mode=="submission_failure")state.submission_error=1;
 if(mode=="no_device"||mode=="init_failure"||mode=="sync_failure"||mode=="submission_failure"){
  throws([&]{lite3_gpu_ndt::make_registration<P>("NDT_CUDA_P2D",1,"DIRECT1");});
  assert(state.gpu_instances==0);return 0;
 }
 auto n=lite3_gpu_ndt::make_registration<P>(mode=="d2d"?"NDT_CUDA_D2D":"NDT_CUDA_P2D",1,"DIRECT1",profile);
 assert(state.gpu_instances==1&&state.mode==(mode=="d2d"?2:1));
 C::Ptr map(new C),src(new C);map->push_back(P{});src->push_back(P{});
 if(mode=="bad_input"){
  src->at(0).x=std::numeric_limits<float>::quiet_NaN();
  throws([&]{n->setInputSource(src);});assert(state.source_uploads==0);return 0;
 }
 n->setInputTarget(map);n->setInputTarget(map);n->setInputSource(src);n->setInputSource(src);
 assert(state.target_uploads==1&&state.source_uploads==1&&state.map_builds==1);
 assert(profile->target_updates==1&&profile->source_updates==1);
 C out;Eigen::Matrix4f seed=Eigen::Matrix4f::Identity();int published=0;
 if(mode=="bad_seed")seed(0,0)=std::numeric_limits<float>::quiet_NaN();
 if(mode=="bad_pose")state.bad_pose=true;
 if(mode=="bad_output")state.bad_output=true;
 if(mode=="runtime_cuda_failure")state.sync_error=1;
 if(mode=="not_converged")state.converged=false;
 auto attempt=[&]{n->align(out,seed);if(n->hasConverged())++published;};
 if(mode=="bad_seed"||mode=="bad_pose"||mode=="bad_output"||mode=="runtime_cuda_failure"){
  throws(attempt);assert(published==0);return 0;
 }
 attempt();assert(published==(mode=="not_converged"?0:1));
 C::Ptr next(new C(*src));n->setInputSource(next);n->align(out,seed);
 assert(state.target_uploads==1&&state.source_uploads==2&&state.map_builds==1);
 assert(state.source_builds==(mode=="d2d"?2:0));
#endif
}
'''


@pytest.fixture(scope='module',params=[('std',False),('std',True),('boost',True)])
def backend_binary(request,tmp_path_factory):
    pointer,cuda=request.param
    d=tmp_path_factory.mktemp('gpu_factory_'+pointer+str(cuda))
    (d/'fake.hpp').write_text(FAKE.replace('PTR<',pointer+'::shared_ptr<'))
    for name in ['pcl/registration/registration.h','pclomp/ndt_omp.h','cuda_runtime_api.h',
                 'fast_gicp/ndt/ndt_cuda.hpp','fast_gicp/cuda/ndt_cuda.cuh']:
        p=d/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include <fake.hpp>\n')
    p=d/'test.cpp';p.write_text(MAIN);exe=d/'test'
    args=['g++','-std=c++14','-O1','-I'+str(d),'-I'+str(ROOT/'experiments/gpu_ndt/include'),
          '-I/usr/include/eigen3',str(p),'-o',str(exe)]
    if cuda:args.insert(1,'-DLITE3_GPU_NDT_ENABLED')
    r=subprocess.run(args,capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    return exe,cuda


@pytest.mark.parametrize('scenario',['cpu','invalid','p2d','d2d','no_device','init_failure',
                         'sync_failure','submission_failure','bad_input','bad_seed',
                         'bad_pose','bad_output','runtime_cuda_failure','not_converged'])
def test_real_factory_and_cuda_wrapper_with_cpp_doubles(backend_binary,scenario):
    exe,cuda=backend_binary
    r=subprocess.run([str(exe),scenario],capture_output=True,text=True)
    assert r.returncode==0,r.stdout+r.stderr


@pytest.fixture
def capture(tmp_path):
    p=tmp_path/'capture';p.mkdir();rows=[]
    pose=np.array([.2,-.3,.4,0,0,0,1.])
    xyz=np.array([[1.,0,.1],[2,.5,.2],[3,-1,.3]],dtype=np.float32)
    for i in range(2):
        np.savez(p/('frame_%02d.npz'%i),published=pose,aligned=xyz+pose[:3],source=xyz)
        rows.append(dict(stamp=100+i*.1))
    (p/'report.json').write_text(json.dumps(rows))
    mapfile=tmp_path/'map.pcd';dataset.write_pcd(mapfile,xyz)
    return p,mapfile,xyz


def test_recorded_dataset_preserves_source_frame_and_timestamp(capture,tmp_path):
    cap,mapfile,xyz=capture
    out=tmp_path/'dataset'
    meta=dataset.prepare(cap,mapfile,out,.1)
    assert meta['cases']==4 and meta['frames'][0]['source_stamp_ns']==100000000000
    for p in out.glob('scan_*.pcd'):
        raw=p.read_bytes().split(b'DATA binary\n',1)[1]
        np.testing.assert_allclose(np.frombuffer(raw,dtype='<f4').reshape(-1,4)[:,:3],xyz,atol=1e-6)
    assert run.verify_dataset(out)==meta
    with pytest.raises(ValueError):dataset.prepare(cap,mapfile,out,.1)


def test_dataset_rotation_is_inverted_without_assuming_planar_pose(capture,tmp_path):
    cap,mapfile,xyz=capture
    q=np.array([.10,-.20,.30,.92]);q/=np.linalg.norm(q)
    pose=np.r_[.2,-.3,.4,q];matrix=dataset.pose_matrix(pose)
    for p in cap.glob('frame_*.npz'):
        np.savez(p,published=pose,aligned=xyz@matrix[:3,:3].T+matrix[:3,3])
    out=tmp_path/'dataset';dataset.prepare(cap,mapfile,out,.1)
    raw=(out/'scan_0000.pcd').read_bytes().split(b'DATA binary\n',1)[1]
    np.testing.assert_allclose(np.frombuffer(raw,dtype='<f4').reshape(-1,4)[:,:3],xyz,atol=1e-6)


@pytest.mark.parametrize('fault',['stamp','quaternion','nonfinite','bad_resolution'])
def test_invalid_recorded_data_refused(capture,tmp_path,fault):
    cap,mapfile,xyz=capture
    if fault=='stamp':(cap/'report.json').write_text(json.dumps([dict(stamp=100),dict(stamp=100)]))
    if fault=='quaternion':np.savez(cap/'frame_00.npz',published=np.zeros(7),aligned=xyz)
    if fault=='nonfinite':np.savez(cap/'frame_00.npz',published=[0,0,0,0,0,0,1],aligned=xyz*np.nan)
    with pytest.raises(ValueError):dataset.prepare(cap,mapfile,tmp_path/'dataset',0 if fault=='bad_resolution' else .1)


def result_fixture(directory,backends=prepare.BACKENDS):
    request=dict(scope='registration_only',dataset_id='same-input',end_to_end_measured=False,
                 backends=list(backends))
    (directory/'request.json').write_text(json.dumps(request))
    (directory/'exit_codes.json').write_text(json.dumps({b:0 for b in backends}))
    for b in backends:
        rows=[dict(kind='metadata',scope='registration_only',backend=b,dataset_id='same-input',
                   resolution=1,map_leaf=.5,search='DIRECT1',expected_cases=1,warmup=3,repeats=1)]
        rows.append(dict(kind='sample',case_id='frame1',trial=0,phase='steady',source_stamp_ns=100,
                         pose=np.eye(4).ravel().tolist(),converged=True,spatial_quality_pass=True,
                         **{k:1. for k in analyze.METRICS}))
        rows.append(dict(kind='summary',finished=True))
        (directory/(b+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))


def test_analysis_reports_metrics_and_does_not_invent_tf_or_fusion(tmp_path):
    result_fixture(tmp_path)
    report=analyze.analyze(tmp_path)
    assert report['status']=='complete'
    assert report['backends']['NDT_CUDA_P2D']['metrics']['align_sync_ms']['p99']==1
    assert report['paired_differences']['NDT_CUDA_P2D']['position_difference_m']['mean']==0
    assert report['tf_and_anchor_continuity']=='未验证'
    assert report['voa_improvement']=='未验证'


@pytest.mark.parametrize('fault',['missing','unfinished','duplicate','negative','error','different_settings','different_input','unmatched','failed_exit'])
def test_incomplete_or_incomparable_results_never_pass(tmp_path,fault):
    result_fixture(tmp_path)
    p=tmp_path/'NDT_CUDA_P2D.jsonl'
    rows=[json.loads(l) for l in p.read_text().splitlines()]
    if fault=='missing':p.unlink()
    elif fault=='unfinished':p.write_text('\n'.join(json.dumps(r) for r in rows[:-1]))
    else:
        if fault=='duplicate':rows.insert(2,dict(rows[1]))
        if fault=='negative':rows[1]['align_sync_ms']=-1
        if fault=='error':rows.insert(2,dict(kind='error',message='CUDA fault'))
        if fault=='different_settings':rows[0]['map_leaf']=.2
        if fault=='different_input':rows[0]['dataset_id']='other'
        if fault=='unmatched':rows[1]['source_stamp_ns']=101
        if fault=='failed_exit':(tmp_path/'exit_codes.json').write_text(json.dumps({b:1 for b in prepare.BACKENDS}))
        p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    if fault in ['different_settings','different_input']:
        with pytest.raises(ValueError):analyze.analyze(tmp_path)
    else:assert analyze.analyze(tmp_path)['status']=='partial'


def test_cpu_only_comparison_is_partial(tmp_path):
    result_fixture(tmp_path,['NDT_OMP'])
    assert analyze.analyze(tmp_path)['status']=='partial'


def test_dataset_tampering_detected(capture,tmp_path):
    cap,mapfile,xyz=capture
    out=tmp_path/'dataset';dataset.prepare(cap,mapfile,out,.1)
    (out/'scan_0000.pcd').write_bytes(b'changed')
    with pytest.raises(ValueError):run.verify_dataset(out)


def test_dataset_can_be_moved_without_editing_cases_or_identity(capture,tmp_path):
    cap,mapfile,xyz=capture
    out=tmp_path/'original';meta=dataset.prepare(cap,mapfile,out,.1)
    rows=[row.split() for row in (out/'cases.tsv').read_text().splitlines()]
    assert all(not Path(row[2]).is_absolute() for row in rows)
    moved=tmp_path/'other_host';shutil.copytree(out,moved)
    assert run.verify_dataset(moved)==meta
    assert all((moved/row[2]).is_file() for row in rows)


def test_runtime_wont_seal_unbuilt_or_tampered_workspace(base,tmp_path):
    out=tmp_path/'out';prepare.prepare(base,out)
    with pytest.raises(ValueError,match='incomplete'):runtime.seal(out)
    (out/'src/hdl_localization/apps/hdl_localization_app.hpp').write_text('changed')
    with pytest.raises(ValueError,match='source changed'):runtime.verify_sources(out)


def test_benchmark_has_no_ros_motion_and_uses_synchronized_boundaries():
    text=(ROOT/'experiments/gpu_ndt/benchmark.cpp').read_text()
    assert 'rclcpp::init' not in text and 'cmd_vel' not in text and 'publish(' not in text
    assert 'new Cloud(*saved)' in text
    assert text.index('registration->align(aligned,c.seed)') < text.index('const double align_ms')
    assert 'full_localization_accepted\\\":null' in text
    assert 'tf_source_age_ms\\\":null' in text


@pytest.mark.parametrize('fault', ['missing_metric','invalid_trial','changed_stamp','invalid_rotation','bad_pose_size'])
def test_analysis_rejects_invalid_samples_without_inventing_a_pose(tmp_path,fault):
    result_fixture(tmp_path)
    p=tmp_path/'NDT_CUDA_P2D.jsonl'
    rows=[json.loads(l) for l in p.read_text().splitlines()]
    if fault=='missing_metric':rows[1].pop('align_sync_ms')
    if fault=='invalid_trial':rows[1]['trial']=10
    if fault=='changed_stamp':rows[1]['source_stamp_ns']=-1
    if fault=='invalid_rotation':rows[1]['pose'][0]=2
    if fault=='bad_pose_size':rows[1]['pose']=[1]
    p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    report=analyze.analyze(tmp_path)
    assert report['status']=='partial'
    assert report['backends']['NDT_CUDA_P2D']['invalid_values']


@pytest.mark.parametrize('fault', ['incomplete','symlink','nested'])
def test_optional_dependency_must_be_an_independent_complete_tree(base,tmp_path,fault):
    dependency=tmp_path/'fast_gicp';dependency.mkdir()
    output=tmp_path/'out'
    for name in ('CMakeLists.txt','package.xml','include/fast_gicp/ndt/ndt_cuda.hpp',
                 'include/fast_gicp/cuda/ndt_cuda.cuh','src/fast_gicp/cuda/ndt_cuda.cu'):
        p=dependency/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text('set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")' if name=='CMakeLists.txt' else 'source')
    if fault=='incomplete':(dependency/'src/fast_gicp/cuda/ndt_cuda.cu').unlink()
    if fault=='symlink':(dependency/'linked').symlink_to(dependency/'package.xml')
    if fault=='nested':output=dependency/'out'
    with pytest.raises(ValueError):prepare.prepare(base,output,dependency)
    assert not output.exists()


def test_runner_executes_only_the_selected_offline_child(capture,tmp_path):
    cap,mapfile,xyz=capture
    data=tmp_path/'dataset';dataset.prepare(cap,mapfile,data,.1)
    binary=tmp_path/'lite3_ndt_benchmark'
    binary.write_text('''#!/usr/bin/env python3
import json,os,sys
from pathlib import Path
Path(sys.argv[4]).write_text(json.dumps(dict(kind="mock_child",backend=sys.argv[1],omp=os.environ["OMP_NUM_THREADS"]))+"\\n")
''')
    binary.chmod(0o700)
    out=tmp_path/'run'
    assert run.run(binary,data,out,['NDT_OMP'],1,1,5)=={'NDT_OMP':0}
    child=json.loads((out/'NDT_OMP.jsonl').read_text())
    assert child==dict(kind='mock_child',backend='NDT_OMP',omp='1')
    assert not (out/'NDT_CUDA_P2D.jsonl').exists()
    resource=json.loads((out/'NDT_OMP.resources.json').read_text())
    assert resource['returncode']==0 and resource['dropped_resource_samples']==0
    with pytest.raises(ValueError,match='already exists'):run.run(binary,data,out,['NDT_OMP'])


def test_runner_times_out_its_child_and_reports_incomplete(capture,tmp_path):
    cap,mapfile,xyz=capture
    data=tmp_path/'dataset';dataset.prepare(cap,mapfile,data,.1)
    binary=tmp_path/'lite3_ndt_benchmark'
    binary.write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(5)\n')
    binary.chmod(0o700)
    out=tmp_path/'run'
    assert run.run(binary,data,out,['NDT_OMP'],1,1,1)=={'NDT_OMP':'timeout'}
    assert analyze.analyze(out)['status']=='partial'


def fake_built_workspace(base,tmp_path,monkeypatch,cuda):
    out=tmp_path/'workspace';prepare.prepare(base,out)
    root=out/'install/hdl_localization/lib/hdl_localization';root.mkdir(parents=True)
    for name in ('hdl_localization_composition','lite3_ndt_benchmark'):(root/name).write_bytes(b'fake binary')
    cache=out/'build/hdl_localization/CMakeCache.txt';cache.parent.mkdir(parents=True)
    cache.write_text('LITE3_GPU_NDT_ENABLED:BOOL='+('ON' if cuda else 'OFF')+'\n')
    monkeypatch.setattr(runtime.subprocess,'run',lambda *args,**kwargs:subprocess.CompletedProcess(args[0],0,'',''))
    runtime.seal(out)
    return out


@pytest.mark.parametrize('cuda', [False,True])
def test_runtime_checks_build_and_domains_without_launching_ros(base,tmp_path,monkeypatch,cuda):
    out=fake_built_workspace(base,tmp_path,monkeypatch,cuda)
    m=tmp_path/'map.pcd';m.write_text('map')
    args,env=runtime.launch_command(out,'NDT_OMP',151,m,.5,.5)
    assert args[0]=='bash' and 'reg_method:=NDT_OMP' in args
    assert env['ROS_DOMAIN_ID']=='151' and env['ROS_LOCALHOST_ONLY']=='1'
    if cuda:
        args,env=runtime.launch_command(out,'NDT_CUDA_D2D',151,m,.5,.5)
        assert 'reg_method:=NDT_CUDA_D2D' in args
    else:
        with pytest.raises(ValueError,match='without CUDA'):runtime.launch_command(out,'NDT_CUDA_P2D',151,m,.5,.5)
    with pytest.raises(ValueError):runtime.launch_command(out,'NDT_OMP',0,m,.5,.5)
    monkeypatch.setenv('ROS_DOMAIN_ID','151')
    with pytest.raises(ValueError):runtime.launch_command(out,'NDT_OMP',151,m,.5,.5)
    with pytest.raises(ValueError,match='already exists'):runtime.seal(out)


def test_sealed_binary_modification_is_refused(base,tmp_path,monkeypatch):
    out=fake_built_workspace(base,tmp_path,monkeypatch,True)
    m=tmp_path/'map.pcd';m.write_text('map')
    (out/'install/hdl_localization/lib/hdl_localization/hdl_localization_composition').write_bytes(b'changed')
    with pytest.raises(ValueError,match='runtime changed'):runtime.launch_command(out,'NDT_CUDA_P2D',151,m,.5,.5)
