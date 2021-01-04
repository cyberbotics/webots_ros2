import sys
from webots_ros2_core.utils import append_webots_python_lib_to_path

try:
    append_webots_python_lib_to_path()
except Exception as e:
    sys.stderr.write('"WEBOTS_HOME" is not correctly set.')
    raise e

#  deepcode ignore C0413: <comment the reason here>
from controller import *
