"""Generate an observation-only standalone benchmark, never the ROS node."""
from pathlib import Path
import shutil

from .. import native
from . import patch


def prepare_source(output,variant):
    output=Path(output).resolve()
    if variant not in 'ABC' or len(variant)!=1:raise ValueError('invalid P2D variant')
    if output.exists():raise ValueError('benchmark output already exists')
    header=(native.HERE/'include/lite3_gpu_ndt/backend.hpp').read_text()
    header=patch.replace_once(header,'#include <chrono>','#include <chrono>\n#include <cstdlib>\n#include <algorithm>')
    header=patch.replace_once(header,'  unsigned source_updates = 0, target_updates = 0;',
        '''  unsigned source_updates = 0, target_updates = 0;
  bool p2d_verify_costs = false;
  unsigned p2d_linearizations = 0, p2d_error_evaluations = 0;
  double p2d_error_evaluation_ms = 0, p2d_max_abs_error = 0, p2d_max_relative_error = 0;''')
    needle='  explicit CheckedCudaNdt(std::shared_ptr<Profile> profile) : profile_(profile) {}'
    header=patch.replace_once(header,needle,'''  explicit CheckedCudaNdt(std::shared_ptr<Profile> profile) : profile_(profile) {
    const char* verify = std::getenv("LITE3_P2D_VERIFY");
    profile_->p2d_verify_costs = verify && std::string(verify)=="1";
    // Built-in LM diagnostics expose rho signs and iteration boundaries;
    // the optimizer code, parameters and branch conditions are unchanged.
    this->setDebugPrint(profile_->p2d_verify_costs);
  }''')
    needle='protected:\n  void computeTransformation('
    header=patch.replace_once(header,needle,'protected:\n'+patch.snippet('profile_methods.inc')+'\n  void computeTransformation(')
    needle='    if (!guess.allFinite()) throw std::invalid_argument("nonfinite CUDA NDT seed");'
    header=patch.replace_once(header,needle,needle+'''
    profile_->p2d_linearizations = profile_->p2d_error_evaluations = 0;
    profile_->p2d_error_evaluation_ms = profile_->p2d_max_abs_error = profile_->p2d_max_relative_error = 0;''')
    source=(native.HERE/'benchmark.cpp').read_text()
    needle='int main(int argc,char** argv) {'
    source=patch.replace_once(source,needle,'''static long long gpu_frequency_hz() {
  std::ifstream input("/sys/devices/17000000.gv11b/devfreq/17000000.gv11b/cur_freq");
  long long value=-1;input>>value;return input?value:-1;
}

'''+needle)
    needle='    pcl::search::KdTree<Point> quality_tree;quality_tree.setInputCloud(map);'
    source=patch.replace_once(source,needle,needle+'''
    const bool verify_costs = profile->p2d_verify_costs;
    if (backend=="NDT_CUDA_D2D") throw std::invalid_argument("P2D experiment excludes D2D");''')
    needle='       <<",\\\"dataset_id\\\":"<<quoted(argv[9])'
    source=patch.replace_once(source,needle,needle+'''
       <<",\\\"p2d_variant\\\":\\\"'''+variant+'''\\\",\\\"p2d_verification_run\\\":"<<(verify_costs?"true":"false")''')
    needle='        Cloud aligned;begin=Clock::now();registration->align(aligned,c.seed);'
    source=patch.replace_once(source,needle,'''        if (verify_costs) std::cout<<"LITE3_CASE "<<c.id<<" "<<trial<<std::endl;
        const long long gpu_hz_before=gpu_frequency_hz();
        const auto align_cpu_begin=std::clock();
'''+needle)
    needle='        const double align_ms=lite3_gpu_ndt::elapsed_ms(begin);'
    source=patch.replace_once(source,needle,needle+'''
        const double align_cpu_ms=1000.0*(std::clock()-align_cpu_begin)/CLOCKS_PER_SEC;
        const long long gpu_hz_after=gpu_frequency_hz();''')
    needle='           <<",\\\"process_cpu_ms\\\":"<<cpu_ms<<",\\\"rss_bytes\\\":"<<rss_bytes()'
    source=patch.replace_once(source,needle,needle+'''
           <<",\\\"lm_linearizations\\\":"<<profile->p2d_linearizations
           <<",\\\"lm_error_evaluations\\\":"<<profile->p2d_error_evaluations
           <<",\\\"align_process_cpu_ms\\\":"<<align_cpu_ms
           <<",\\\"gpu_frequency_before_hz\\\":";
        if(gpu_hz_before>=0)out<<gpu_hz_before;else out<<"null";
        out<<",\\\"gpu_frequency_after_hz\\\":";
        if(gpu_hz_after>=0)out<<gpu_hz_after;else out<<"null";
        out
           <<",\\\"candidate_error_ms\\\":"<<profile->p2d_error_evaluation_ms
           <<",\\\"scalar_reference_max_abs\\\":"<<profile->p2d_max_abs_error
           <<",\\\"scalar_reference_max_relative\\\":"<<profile->p2d_max_relative_error''')
    output.mkdir(parents=True)
    for name in ['CMakeLists.txt','targets.cmake']:shutil.copy2(native.HERE/name,output/name)
    p=output/'include/lite3_gpu_ndt/backend.hpp';p.parent.mkdir(parents=True);p.write_text(header)
    (output/'benchmark.cpp').write_text(source)
    return output
