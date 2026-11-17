"""Tool and verification-gate tests with FAKE results; no native CUDA runs."""
import json
from pathlib import Path
import subprocess
import sys
import shlex

import pytest

from experiments.gpu_ndt import native,prepare
from experiments.gpu_ndt.p2d_optimization import benchmark_source,experiment,patch,report,verify
from test_gpu_ndt_experiment import FAKE


def result(path,variant='A',*,verification=True):
    path.mkdir()
    backend=verify.BACKEND
    request=dict(scope='registration_only',backends=[backend],dataset_id='fixture-dataset')
    (path/'request.json').write_text(json.dumps(request))
    (path/'exit_codes.json').write_text(json.dumps({backend:0}))
    meta=dict(kind='metadata',scope='registration_only',backend=backend,dataset_id='fixture-dataset',
              p2d_variant=variant,p2d_verification_run=verification,expected_cases=1,warmup=1,repeats=1,
              resolution=1.0,map_leaf=.1,search='DIRECT1',max_iterations=64,transformation_epsilon=.0005)
    rows=[meta,dict(kind='device',compute_major=7,compute_minor=2)]
    for trial in [-1,0]:
        row=dict(kind='sample',phase='warmup' if trial<0 else 'steady',case_id='frame0',trial=trial,source_stamp_ns=12345,
                 pose=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],lm_linearizations=1,lm_error_evaluations=2,
                 scalar_reference_max_abs=0.,scalar_reference_max_relative=0.,candidate_error_ms=1.,align_process_cpu_ms=.5,
                 converged=True,spatial_quality_pass=True,quality_mse=.02,overlap=.9)
        for k in ['prepare_ms','source_set_sync_ms','quality_ms','registration_pipeline_ms','process_cpu_ms','rss_bytes','align_sync_ms']:
            row[k]=1.
        rows.append(row)
    rows.append(dict(kind='summary',finished=True))
    (path/(backend+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (path/(backend+'.stderr.log')).write_text(''.join(
        'LITE3_CASE frame0 '+str(trial)+'\n--- LM optimization ---\n'
        '    i y0 yi rho lambda |delta| dec\n'
        '    0 1 2 -1 1e-9 .1\n'
        '    1 1 .1 1 1e-8 .01 x\n' for trial in [-1,0]))
    return path


def edit_rows(path,fn):
    f=path/(verify.BACKEND+'.jsonl')
    rows=[json.loads(r) for r in f.read_text().splitlines()]
    fn(rows);f.write_text(''.join(json.dumps(r)+'\n' for r in rows))


@pytest.mark.parametrize('variant',list('ABC'))
def test_generate_probe_is_separate_and_keeps_optimizer_and_cpu_parameters(tmp_path,variant):
    before=(native.HERE/'include/lite3_gpu_ndt/backend.hpp').read_bytes()
    out=benchmark_source.prepare_source(tmp_path/'probe',variant)
    header=(out/'include/lite3_gpu_ndt/backend.hpp').read_text()
    cpp=(out/'benchmark.cpp').read_text()
    assert before==(native.HERE/'include/lite3_gpu_ndt/backend.hpp').read_bytes()
    assert 'return Base::linearize(trans,H,b);' in header
    assert 'const double error = Base::compute_error(trans);' in header
    assert 'if (profile_->p2d_verify_costs)' in header
    assert 'ndt_cuda_->compute_error(trans,&H,&b)' in header
    assert 'rho =' not in header and 'LDLT' not in header
    for line in ['ndt->setTransformationEpsilon(0.01);','pclomp::DIRECT1 : pclomp::DIRECT7',
                 'new pclomp::NormalDistributionsTransform<Point, Point>()']:
        assert line in header
    assert 'LITE3_CASE' in cpp and 'scalar_reference_max_abs' in cpp
    assert '\\"p2d_variant\\":\\"'+variant+'\\"' in cpp


def test_probe_cannot_overwrite_an_existing_tree(tmp_path):
    with pytest.raises(ValueError,match='exists'):benchmark_source.prepare_source(tmp_path,'A')


def test_native_build_requires_complete_actual_source_not_reference_bundle(tmp_path):
    out=tmp_path/'prepared';patch.prepare(patch.HERE/'baseline',out,'B')
    with pytest.raises(ValueError,match='complete installed'):experiment.checked_variant(out)


def test_prepare_rejects_external_source_symlink_before_copy(tmp_path):
    base=tmp_path/'base';patch.prepare(patch.HERE/'baseline',base,'A')
    (base/'external_link').symlink_to('/etc/hosts')
    with pytest.raises(ValueError,match='symlinks'):patch.prepare(base,tmp_path/'new','C')
    assert not (tmp_path/'new').exists()


def test_changed_unrelated_source_rejected_before_native_build(tmp_path):
    out=tmp_path/'prepared';patch.prepare(patch.HERE/'baseline',out,'B')
    (out/'extra.cpp').write_text('unrecorded source')
    with pytest.raises(ValueError,match='complete prepared source inventory changed'):experiment.checked_variant(out)


def test_native_c_requires_real_b_gate_before_writing(tmp_path,monkeypatch):
    monkeypatch.setattr(native,'idle_required',lambda confirmed:None)
    monkeypatch.setattr(experiment,'checked_variant',lambda p:dict(variant='C'))
    with pytest.raises(ValueError,match='requires a passed A/B'):
        experiment.build(tmp_path/'C',tmp_path/'source',tmp_path/'ndt',tmp_path/'official',True)
    assert list(tmp_path.iterdir())==[]


def test_fake_or_incomplete_native_receipt_rejected(tmp_path):
    p=tmp_path/'receipt';p.write_text(json.dumps(dict(passed=True,variant='B')))
    with pytest.raises(ValueError,match='invalid A/B'):verify.require_b_receipt(p)


@pytest.mark.parametrize('action',['build','run'])
def test_operator_confirmation_is_checked_before_files(tmp_path,action):
    with pytest.raises(ValueError,match='confirmation'):
        if action=='build':experiment.build(tmp_path/'root',tmp_path/'source',tmp_path/'ndt',tmp_path/'official')
        else:experiment.run_variant(tmp_path/'root',tmp_path/'dataset',tmp_path/'output','verify')
    assert list(tmp_path.iterdir())==[]


def test_native_verification_mock_fixture_accepts_identical_sequences(tmp_path):
    a=result(tmp_path/'A');b=result(tmp_path/'B','B')
    receipt=verify.compare(a,b)
    assert receipt['passed'] and receipt['samples']==2 and receipt['variant']=='B'
    assert receipt['performance_measured'] is False


@pytest.mark.parametrize('field,value',[
    ('pose',[1,0,0,.01,0,1,0,0,0,0,1,0,0,0,0,1]),
    ('pose',[float('nan')]*16),
    ('lm_error_evaluations',3),('lm_linearizations',2),
    ('converged',False),('spatial_quality_pass',False),('quality_mse',.05),
    ('overlap',.8),('scalar_reference_max_relative',.01)])
def test_verification_stops_on_native_result_regressions_in_mock_data(tmp_path,field,value):
    a=result(tmp_path/'A');b=result(tmp_path/'B','B')
    edit_rows(b,lambda rows:rows[3].update({field:value}))
    with pytest.raises(ValueError):verify.compare(a,b)


def test_rho_sign_sequence_difference_is_not_hidden_by_identical_final_pose(tmp_path):
    a=result(tmp_path/'A');b=result(tmp_path/'B','B')
    log=b/(verify.BACKEND+'.stderr.log')
    log.write_text(log.read_text().replace('1 2 -1','1 2 1'))
    with pytest.raises(ValueError,match='rho-sign'):verify.compare(a,b)


def test_zero_rho_uses_actual_rejection_condition_instead_of_debug_x_marker(tmp_path):
    p=tmp_path/'trace.log'
    p.write_text('LITE3_CASE f 0\n0 1 1 0 .001 .0001\n')
    assert verify.lm_trace(p)=={('f',0):[(0,False)]}


def test_verification_requires_full_trace_and_samples(tmp_path):
    a=result(tmp_path/'A');b=result(tmp_path/'B','B')
    edit_rows(b,lambda rows:rows.pop(3))
    with pytest.raises(ValueError,match='missing verification samples'):verify.compare(a,b)


def test_plain_timing_output_cannot_pass_numerical_verification(tmp_path):
    a=result(tmp_path/'A');b=result(tmp_path/'B','B',verification=False)
    with pytest.raises(ValueError,match='timing/Mock'):verify.compare(a,b)


def test_numerical_debug_timings_cannot_be_reported_as_performance(tmp_path):
    a=result(tmp_path/'A');b=result(tmp_path/'B','B');c=result(tmp_path/'C','C')
    with pytest.raises(ValueError,match='verification/debug'):report.summarize(a,b,c)


def test_performance_report_preserves_counts_missing_frequencies_and_scope(tmp_path):
    dirs=[result(tmp_path/v,v,verification=False) for v in 'ABC']
    summary=report.summarize(*dirs)
    assert summary['backends']['B']['metrics']['lm_error_evaluations']['mean']==2
    assert summary['paired_with_original_p2d']['C']['lm_counts_equal_to_A'] is True
    assert summary['coarse_mean_gpu_frequencies_within_5_percent'] is False
    assert summary['full_localization_and_voa']=='未验证'


def test_api_counts_are_whole_trace_not_per_frame(tmp_path):
    p=tmp_path/'api.csv'
    p.write_text('Time (%),Total Time (ns),Num Calls,Name\n50,10000000,12,cudaMalloc\n50,30000000,30,cudaStreamSynchronize\n')
    result=report.api_totals(p)
    assert result['apis']['cudaMalloc']==dict(calls=12,total_time_ms=10.)
    assert 'whole trace' in result['scope']


def test_new_kernel_evidence_is_required():
    with pytest.raises(ValueError,match='own Nsight'):experiment.error_kernel_evidence(None,{})


@pytest.mark.parametrize('fault',['none','only_old_kernel','wrong_binary','changed_report'])
def test_error_kernel_gate_checks_executed_events_and_trace_identity_with_mock_files(tmp_path,fault):
    runtime=dict(root=str(tmp_path),binary=str(tmp_path/'build/lite3_ndt_benchmark'))
    trace=tmp_path/'traces/error_only';trace.mkdir(parents=True)
    rep=trace/'trace.nsys-rep';rep.write_bytes(b'FAKE test fixture, not native Nsight')
    evidence=trace/'evidence.json'
    evidence.write_text(json.dumps(dict(nsys_report=str(rep),report_sha256=native.sha(rep),
                                      backend=verify.BACKEND,cuda_kernel_executed=True)))
    binary=runtime['binary'] if fault!='wrong_binary' else '/another/benchmark'
    (trace/'request.json').write_text(json.dumps(dict(command=['nsys','profile',binary])))
    kernel='p2d_ndt_error_only_kernel' if fault!='only_old_kernel' else 'p2d_ndt_compute_derivatives_kernel'
    (trace/'kernel_summary.csv').write_text('Name,Instances\n'+kernel+',3\n')
    if fault=='changed_report':rep.write_bytes(b'tampered fixture')
    if fault=='none':assert str(rep) in experiment.error_kernel_evidence(evidence,runtime)
    else:
        with pytest.raises(ValueError):experiment.error_kernel_evidence(evidence,runtime)


@pytest.mark.parametrize('module',['patch','experiment','report'])
def test_cli_help_does_not_build_or_run_cuda(module):
    p=subprocess.run([sys.executable,'-m','experiments.gpu_ndt.p2d_optimization.'+module,'--help'],
                     capture_output=True,text=True,timeout=10)
    assert p.returncode==0,p.stderr


def test_no_control_deployment_or_frequency_commands_in_new_execution_tool():
    source=Path(experiment.__file__).read_text()
    for token in ["'sudo'","'ssh'","'ros2'","'jetson_clocks'","'nvpmodel'","'pkill'"]:
        assert token not in source
    assert 'native.build(' in source and 'benchmark_source=bench' in source


def test_known_sm72_cmake_flags_are_not_duplicated():
    # Check both accepted build-input forms; production build flags are untouched.
    source=Path(native.__file__).read_text()
    assert 'cmake.count(known_sm72)==1' in source
    original='set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")'
    assert prepare.patch_cuda_arch(original).count('code=sm_72')==1


@pytest.fixture(scope='module',params=['std','boost'])
def probe_binary(request,tmp_path_factory):
    """Compile actual generated CUDA-wrapper C++ with explicit API doubles."""
    root=tmp_path_factory.mktemp('probe_'+request.param)
    source=benchmark_source.prepare_source(root/'source','B')
    fake=FAKE.replace('PTR<',request.param+'::shared_ptr<')
    fake=fake.replace(' int devices=1,',' double candidate=1.,reference=1.;int ref_calls=0;bool debug=false;\n int devices=1,')
    fake=fake.replace(' bool target=false,source=false;', ''' bool target=false,source=false;
 double compute_error(const Eigen::Isometry3d&,Eigen::Matrix<double,6,6>* H,Eigen::Matrix<double,6,1>* b) const {
  ++state.ref_calls;*H=Eigen::Matrix<double,6,6>::Identity();*b=Eigen::Matrix<double,6,1>::Zero();return state.reference;
 }''')
    fake=fake.replace(' NDTCuda(){++state.gpu_instances;}', ''' NDTCuda(){++state.gpu_instances;}
 void setDebugPrint(bool enabled){state.debug=enabled;}
 virtual double linearize(const Eigen::Isometry3d&,Eigen::Matrix<double,6,6>* H,Eigen::Matrix<double,6,1>* b) {
  *H=Eigen::Matrix<double,6,6>::Identity();*b=Eigen::Matrix<double,6,1>::Zero();return 1.;
 }
 virtual double compute_error(const Eigen::Isometry3d&){return state.candidate;}''')
    (root/'fake.hpp').write_text(fake)
    for name in ['pcl/registration/registration.h','pclomp/ndt_omp.h','cuda_runtime_api.h',
                 'fast_gicp/ndt/ndt_cuda.hpp','fast_gicp/cuda/ndt_cuda.cuh']:
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include <fake.hpp>\n')
    code=r'''
#include <lite3_gpu_ndt/backend.hpp>
#include <cassert>
struct Probe:lite3_gpu_ndt::CheckedCudaNdt<pcl::PointXYZI> {
 using Parent=lite3_gpu_ndt::CheckedCudaNdt<pcl::PointXYZI>;
 using Parent::Parent;using Parent::compute_error;using Parent::linearize;
};
int main(int argc,char** argv) {
 std::string mode=argv[1];
 setenv("LITE3_P2D_VERIFY",mode=="perf"?"0":"1",1);
 auto profile=std::make_shared<lite3_gpu_ndt::Profile>();Probe p(profile);
 Eigen::Matrix<double,6,6> H;Eigen::Matrix<double,6,1> b;
 p.linearize(Eigen::Isometry3d::Identity(),&H,&b);
 assert(profile->p2d_linearizations==1);
 if(mode=="mismatch")state.reference=2.;
 if(mode=="nan")state.candidate=std::numeric_limits<double>::quiet_NaN();
 bool thrown=false;try{p.compute_error(Eigen::Isometry3d::Identity());}catch(const std::exception&){thrown=true;}
 assert(thrown==(mode=="mismatch"||mode=="nan"));
 assert(profile->p2d_error_evaluations==1);
 if(mode=="perf")assert(state.ref_calls==0&&!state.debug);
 if(mode=="verify")assert(state.ref_calls==1&&state.debug);
 return 0;
}
'''
    cpp=root/'probe.cpp';cpp.write_text(code);binary=root/'probe'
    p=subprocess.run(['g++','-std=c++14','-O1','-DLITE3_GPU_NDT_ENABLED','-I'+str(root),
                      '-I'+str(source/'include'),'-I/usr/include/eigen3',str(cpp),'-o',str(binary)],
                     capture_output=True,text=True,timeout=60)
    assert p.returncode==0,p.stderr
    return binary


@pytest.mark.parametrize('scenario',['perf','verify','mismatch','nan'])
def test_actual_probe_wrapper_cpp_with_cuda_api_doubles(probe_binary,scenario):
    p=subprocess.run([str(probe_binary),scenario],capture_output=True,text=True,timeout=10)
    assert p.returncode==0,p.stdout+p.stderr


def test_full_generated_benchmark_cpp_syntax_with_real_pcl_and_cpu_ndt_double(tmp_path):
    source=benchmark_source.prepare_source(tmp_path/'source','C')
    p=tmp_path/'pclomp/ndt_omp.h';p.parent.mkdir()
    p.write_text('''#pragma once
#include <pcl/registration/registration.h>
namespace pclomp {
enum Search{DIRECT1,DIRECT7};
template<class P,class T>struct NormalDistributionsTransform:pcl::Registration<P,T> {
 using Ptr=pcl::shared_ptr<NormalDistributionsTransform<P,T>>;
 void setTransformationEpsilon(double){} void setResolution(double){}
 void setNeighborhoodSearchMethod(Search){} double getStepSize(){return .1;}
 void computeTransformation(pcl::PointCloud<P>&,const typename pcl::Registration<P,T>::Matrix4&)override{}
};
}
''')
    flags=subprocess.run(['pkg-config','--cflags','pcl_io','pcl_filters','pcl_registration'],capture_output=True,text=True)
    if flags.returncode:pytest.skip('PCL development headers unavailable; no GPU syntax result implied')
    p=subprocess.run(['g++','-std=c++14','-fsyntax-only','-I'+str(tmp_path),'-I'+str(source/'include'),
                      *shlex.split(flags.stdout),str(source/'benchmark.cpp')],capture_output=True,text=True,timeout=90)
    assert p.returncode==0,p.stderr
