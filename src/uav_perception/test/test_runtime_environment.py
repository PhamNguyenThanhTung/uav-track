"""Check the production package's dependency-installation boundary."""
import os
import subprocess
import sys


def test_project_import_disables_ultralytics_auto_install():
    result = subprocess.run(
        [sys.executable, '-c',
         'import uav_perception; from ultralytics.utils import AUTOINSTALL; '
         'assert AUTOINSTALL is False'],
        env={**os.environ, 'YOLO_AUTOINSTALL': 'true'},
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
