"""Compilation must not be mistaken for a live sensor/localization process."""
from pathlib import Path
import re,subprocess,unittest
class ProcessGuardTests(unittest.TestCase):
    def test_compiler_ignored_real_nodes_and_replay_detected(self):
        source=(Path(__file__).resolve().parents[1]/'navigation/start_localization_only.sh').read_text()
        pattern=re.search(r"if pgrep -af '([^']+)'",source).group(1)
        cases={'colcon build --packages-select hdl_localization':False,
               'cc1plus /ws/apps/hdl_localization_composition.cpp -o /tmp/test.s':False,
               '/ws/install/hdl_localization/lib/hdl_localization/hdl_localization_composition --ros-args':True,
               '/ws/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node':True,
               '/usr/bin/python3 /opt/ros/foxy/bin/ros2 bag play data':True}
        for command,expected in cases.items():
            with self.subTest(command=command):
                result=subprocess.run(['grep','-E',pattern],input=command+'\n',text=True,capture_output=True)
                self.assertEqual(result.returncode==0,expected)
