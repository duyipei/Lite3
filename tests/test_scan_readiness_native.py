from pathlib import Path
import subprocess


def test_exact_tf_can_arrive_late_without_reusing_old_pose(tmp_path):
    root=Path(__file__).resolve().parents[1]
    src=(root/'vendor_patch/hdl_localization_app.hpp').read_text()
    method=src.split('// BEGIN_SCAN_READINESS')[1].split('// END_SCAN_READINESS')[0]
    code='#include <cmath>\n#include <cassert>\n'+method+r'''
int main(){
 assert(ScanReadiness(10,10.005,9.9,false)==ScanDecision::Wait);
 assert(ScanReadiness(10,10.045,9.9,true)==ScanDecision::Process);
 assert(ScanReadiness(10,10.1,10,true)==ScanDecision::Drop);
 assert(ScanReadiness(10,10.51,9.9,false)==ScanDecision::Drop);
 assert(ScanReadiness(10,10.51,9.9,true)==ScanDecision::Drop);
 assert(ScanReadiness(20,10,9.9,true)==ScanDecision::Drop);
}
'''
    p=tmp_path/'scan.cpp';p.write_text(code);exe=p.with_suffix('')
    subprocess.run(['g++','-std=c++14',str(p),'-o',str(exe)],check=True,capture_output=True)
    subprocess.run([str(exe)],check=True)
