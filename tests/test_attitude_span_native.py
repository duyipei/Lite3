"""Run the production 3D stability check on captured failures and sign-equivalent quaternions."""
from pathlib import Path
import json,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class AttitudeSpanTests(unittest.TestCase):
 def test_pitch_drift_rejected_without_rejecting_quaternion_sign(self):
  src=(ROOT/'vendor_patch/hdl_localization_app.hpp').read_text()
  method=src.split('// BEGIN_ROTATION_SPAN')[1].split('// END_ROTATION_SPAN')[0]
  records=json.loads((ROOT/'results/ground_acceptance_20261028/feedback_settled/report.json').read_text())
  poses=[[r['candidate'][6]]+r['candidate'][3:6] for r in records]
  init=','.join('{'+','.join(str(x) for x in q)+'}' for q in poses)
  code='#include <Eigen/Geometry>\n#include <array>\n#include <deque>\n#include <limits>\n#include <cassert>\n#include <iostream>\n'+method+'\nint main(){std::deque<std::array<double,4>> captured={'+init+'};'
  code+='\n const auto span=RotationSpan(captured);std::cout<<span<<std::endl;assert(span>.04363323);\n std::deque<std::array<double,4>> steady={{{1,0,0,0}},{{-1,0,0,0}},{{1,0,.001,0}},{{-1,0,-.001,0}}};\n assert(RotationSpan(steady)<.003);assert(!std::isfinite(RotationSpan({})));\n assert(!std::isfinite(RotationSpan({{{0,0,0,0}}})));\n assert(!std::isfinite(RotationSpan({{{std::numeric_limits<double>::quiet_NaN(),0,0,0}}})));\n}\n'
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'span.cpp';c.write_text(code);exe=c.with_suffix('')
   subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3',str(c),'-o',str(exe)],check=True,capture_output=True)
   subprocess.run([str(exe)],check=True,capture_output=True)
