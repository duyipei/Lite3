"""Execute real session traps with cleanup stubbed; no ROS, SSH or robot."""
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('ending,status,event', [
    ('false',1,'shell_exit'),
    ('kill -INT $$',130,'signal_INT'),
    ('kill -TERM $$',143,'signal_TERM'),
    ('kill -HUP $$',129,'signal_HUP'),
    ('exit 0',0,'shell_exit')])
def test_actual_traps_preserve_exit_status_and_record_why(tmp_path,ending,status,event):
    source = (ROOT/'scripts/navigation_start.sh').read_text()
    fragment = 'session_event() {'+source.split('session_event() {',1)[1].split('\nshow_failure_logs()',1)[0]
    command = ('set -Eeuo pipefail\nLOG_DIR=$1\nSESSION_ACTIVE=1\nLAST_CONTROL_COMMAND=CLEAR\n'
               'stop_local_sessions() { printf cleaned >> "$LOG_DIR/cleanup_called"; }\n'
               +fragment+'\n'+ending+'\n')
    result = subprocess.run(['bash','-c',command,'fixture',str(tmp_path)],capture_output=True,text=True,timeout=5)
    assert result.returncode == status
    log = (tmp_path/'session_events.log').read_text()
    assert 'event='+event+' status='+str(status)+' control=CLEAR' in log
    assert (tmp_path/'cleanup_called').read_text() == 'cleaned'
    if ending == 'false':
        assert 'command_error_line_' in log
