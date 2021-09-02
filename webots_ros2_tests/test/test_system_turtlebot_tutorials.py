#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test the `webots_ros2_turtlebot` package on the SLAM and Navigation tutorials."""

# Launch the test locally: launch_test src/webots_ros2/webots_ros2_tests/test/test_system_turtlebot_tutorials.py

import os
import sys
import pytest
import rclpy
from cartographer_ros_msgs.msg import SubmapList
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from launch import LaunchDescription
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_tests.utils import TestWebots, initialize_webots_test



@pytest.mark.rostest
def generate_test_description():
    initialize_webots_test()

    # Webots
    turtlebot_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_turtlebot'), 'launch', 'robot_launch.py')
        )
    )
    
    # Rviz SLAM
    turtlebot_SLAM = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    #Rviz Navigation
    os.environ["TURTLEBOT3_MODEL"] = "burger"
    
    turtlebot_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true',
        'map': os.path.join(get_package_share_directory('webots_ros2_turtlebot'), 'resource', 'turtlebot3_burger_example_map.yaml')}.items(),
    )

    return LaunchDescription([
        turtlebot_webots,
        turtlebot_SLAM,
        turtlebot_navigation,
        launch_testing.actions.ReadyToTest(),
    ])
        

class TestTurtlebotTutorials(TestWebots):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.__node = rclpy.create_node('driver_tester')
        self.wait_for_clock(self.__node, messages_to_receive=20)
        
    def testSLAM(self):
        def on_map_message_received(message):
            # There should be an update of the submap
            update_found = 0
            for submap in message.submap:
                if submap.submap_version > 1:
                    update_found = 1
            return update_found

        self.wait_for_messages(self.__node, SubmapList, '/submap_list', condition=on_map_message_received)

    def testNavigation(self):
        # Set the initial pose
        publisher = self.__node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        pose_message = PoseWithCovarianceStamped()
        pose_message.header.frame_id = "map"

        initial_point = Point()
        initial_point.x = 0.0
        initial_point.y = 0.0
        initial_point.z = 0.0
        pose_message.pose.pose.position = initial_point

        initial_orientation = Quaternion()
        initial_orientation.x = 0.0
        initial_orientation.y = 0.0
        initial_orientation.z = 0.0
        initial_orientation.w = 1.0
        pose_message.pose.pose.orientation = initial_orientation

        # Wait for Webots before sending the message
        self.wait_for_clock(self.__node)
        publisher.publish(pose_message)

        # Check if the cost map is updated -> local map for navigation is working
        def on_cost_map_message_received(message):
            return 1

        self.wait_for_messages(self.__node, OccupancyGrid, '/global_costmap/costmap',condition=on_cost_map_message_received)

    def tearDown(self):
        self.__node.destroy_node()


'sys' present in sys.modules 

'builtins' present in sys.modules 

'_frozen_importlib' present in sys.modules 

'_imp' present in sys.modules 

'_warnings' present in sys.modules 

'_io' present in sys.modules 

'marshal' present in sys.modules 

'posix' present in sys.modules 

'_frozen_importlib_external' present in sys.modules 

'_thread' present in sys.modules 

'_weakref' present in sys.modules 

'time' present in sys.modules 

'zipimport' present in sys.modules 

'_codecs' present in sys.modules 

'codecs' present in sys.modules 

'encodings.aliases' present in sys.modules 

'encodings' present in sys.modules 

'encodings.utf_8' present in sys.modules 

'_signal' present in sys.modules 

'__main__' present in sys.modules 

'encodings.latin_1' present in sys.modules 

'_abc' present in sys.modules 

'abc' present in sys.modules 

'io' present in sys.modules 

'_stat' present in sys.modules 

'stat' present in sys.modules 

'_collections_abc' present in sys.modules 

'genericpath' present in sys.modules 

'posixpath' present in sys.modules 

'os.path' present in sys.modules 

'os' present in sys.modules 

'_sitebuiltins' present in sys.modules 

'_locale' present in sys.modules 

'_bootlocale' present in sys.modules 

'types' present in sys.modules 

'importlib._bootstrap' present in sys.modules 

'importlib._bootstrap_external' present in sys.modules 

'warnings' present in sys.modules 

'importlib' present in sys.modules 

'importlib.machinery' present in sys.modules 

'importlib.abc' present in sys.modules 

'_operator' present in sys.modules 

'operator' present in sys.modules 

'keyword' present in sys.modules 

'_heapq' present in sys.modules 

'heapq' present in sys.modules 

'itertools' present in sys.modules 

'reprlib' present in sys.modules 

'_collections' present in sys.modules 

'collections' present in sys.modules 

'_functools' present in sys.modules 

'functools' present in sys.modules 

'contextlib' present in sys.modules 

'importlib.util' present in sys.modules 

'mpl_toolkits' present in sys.modules 

'zope' present in sys.modules 

'apport_python_hook' present in sys.modules 

'sitecustomize' present in sys.modules 

'site' present in sys.modules 

'enum' present in sys.modules 

'_sre' present in sys.modules 

'sre_constants' present in sys.modules 

'sre_parse' present in sys.modules 

'sre_compile' present in sys.modules 

'copyreg' present in sys.modules 

're' present in sys.modules 

'__future__' present in sys.modules 

'binascii' present in sys.modules 

'fnmatch' present in sys.modules 

'errno' present in sys.modules 

'zlib' present in sys.modules 

'_compression' present in sys.modules 

'_weakrefset' present in sys.modules 

'threading' present in sys.modules 

'_bz2' present in sys.modules 

'bz2' present in sys.modules 

'_lzma' present in sys.modules 

'lzma' present in sys.modules 

'pwd' present in sys.modules 

'grp' present in sys.modules 

'shutil' present in sys.modules 

'_struct' present in sys.modules 

'struct' present in sys.modules 

'zipfile' present in sys.modules 

'weakref' present in sys.modules 

'pkgutil' present in sys.modules 

'platform' present in sys.modules 

'math' present in sys.modules 

'_datetime' present in sys.modules 

'datetime' present in sys.modules 

'xml' present in sys.modules 

'xml.parsers' present in sys.modules 

'pyexpat.errors' present in sys.modules 

'pyexpat.model' present in sys.modules 

'pyexpat' present in sys.modules 

'xml.parsers.expat.model' present in sys.modules 

'xml.parsers.expat.errors' present in sys.modules 

'xml.parsers.expat' present in sys.modules 

'plistlib' present in sys.modules 

'email' present in sys.modules 

'email.errors' present in sys.modules 

'_string' present in sys.modules 

'string' present in sys.modules 

'email.quoprimime' present in sys.modules 

'base64' present in sys.modules 

'email.base64mime' present in sys.modules 

'quopri' present in sys.modules 

'email.encoders' present in sys.modules 

'email.charset' present in sys.modules 

'email.header' present in sys.modules 

'_bisect' present in sys.modules 

'bisect' present in sys.modules 

'_sha512' present in sys.modules 

'_random' present in sys.modules 

'random' present in sys.modules 

'_socket' present in sys.modules 

'collections.abc' present in sys.modules 

'select' present in sys.modules 

'selectors' present in sys.modules 

'socket' present in sys.modules 

'urllib' present in sys.modules 

'urllib.parse' present in sys.modules 

'locale' present in sys.modules 

'calendar' present in sys.modules 

'email._parseaddr' present in sys.modules 

'email.utils' present in sys.modules 

'email._policybase' present in sys.modules 

'email.feedparser' present in sys.modules 

'email.parser' present in sys.modules 

'tempfile' present in sys.modules 

'textwrap' present in sys.modules 

'_opcode' present in sys.modules 

'opcode' present in sys.modules 

'dis' present in sys.modules 

'token' present in sys.modules 

'tokenize' present in sys.modules 

'linecache' present in sys.modules 

'inspect' present in sys.modules 

'ntpath' present in sys.modules 

'pkg_resources.extern' present in sys.modules 

'pkg_resources._vendor' present in sys.modules 

'pkg_resources.extern.six' present in sys.modules 

'pkg_resources._vendor.six' present in sys.modules 

'pkg_resources.extern.six.moves' present in sys.modules 

'pkg_resources._vendor.six.moves' present in sys.modules 

'pkg_resources.py31compat' present in sys.modules 

'pkg_resources.extern.appdirs' present in sys.modules 

'pkg_resources._vendor.packaging.__about__' present in sys.modules 

'pkg_resources.extern.packaging' present in sys.modules 

'pkg_resources.extern.packaging._structures' present in sys.modules 

'pkg_resources.extern.packaging.version' present in sys.modules 

'pkg_resources.extern.packaging._compat' present in sys.modules 

'pkg_resources.extern.packaging.specifiers' present in sys.modules 

'copy' present in sys.modules 

'pprint' present in sys.modules 

'traceback' present in sys.modules 

'pkg_resources.extern.pyparsing' present in sys.modules 

'pkg_resources.extern.six.moves.urllib' present in sys.modules 

'pkg_resources.extern.packaging.markers' present in sys.modules 

'pkg_resources.extern.packaging.requirements' present in sys.modules 

'pkg_resources.py2_warn' present in sys.modules 

'sysconfig' present in sys.modules 

'pkg_resources' present in sys.modules 

'osrf_pycommon' present in sys.modules 

'osrf_pycommon.terminal_color.ansi_re' present in sys.modules 

'osrf_pycommon.terminal_color.impl' present in sys.modules 

'osrf_pycommon.terminal_color' present in sys.modules 

'launch_testing.tools.text' present in sys.modules 

'launch_testing.tools.output' present in sys.modules 

'typing.io' present in sys.modules 

'typing.re' present in sys.modules 

'typing' present in sys.modules 

'atexit' present in sys.modules 

'logging' present in sys.modules 

'_compat_pickle' present in sys.modules 

'_pickle' present in sys.modules 

'pickle' present in sys.modules 

'_queue' present in sys.modules 

'queue' present in sys.modules 

'logging.handlers' present in sys.modules 

'launch.logging.handlers' present in sys.modules 

'launch.logging' present in sys.modules 

'concurrent' present in sys.modules 

'concurrent.futures._base' present in sys.modules 

'concurrent.futures' present in sys.modules 

'signal' present in sys.modules 

'_posixsubprocess' present in sys.modules 

'subprocess' present in sys.modules 

'_ssl' present in sys.modules 

'ssl' present in sys.modules 

'asyncio.constants' present in sys.modules 

'asyncio.format_helpers' present in sys.modules 

'asyncio.base_futures' present in sys.modules 

'asyncio.log' present in sys.modules 

'asyncio.coroutines' present in sys.modules 

'_contextvars' present in sys.modules 

'contextvars' present in sys.modules 

'asyncio.exceptions' present in sys.modules 

'asyncio.base_tasks' present in sys.modules 

'_asyncio' present in sys.modules 

'asyncio.events' present in sys.modules 

'asyncio.futures' present in sys.modules 

'asyncio.protocols' present in sys.modules 

'asyncio.transports' present in sys.modules 

'asyncio.sslproto' present in sys.modules 

'asyncio.locks' present in sys.modules 

'asyncio.tasks' present in sys.modules 

'asyncio.staggered' present in sys.modules 

'asyncio.trsock' present in sys.modules 

'asyncio.base_events' present in sys.modules 

'asyncio.runners' present in sys.modules 

'asyncio.queues' present in sys.modules 

'asyncio.streams' present in sys.modules 

'asyncio.subprocess' present in sys.modules 

'asyncio.base_subprocess' present in sys.modules 

'asyncio.selector_events' present in sys.modules 

'asyncio.unix_events' present in sys.modules 

'asyncio' present in sys.modules 

'launch.event' present in sys.modules 

'launch.launch_description_entity' present in sys.modules 

'launch.some_actions_type' present in sys.modules 

'launch.event_handler' present in sys.modules 

'launch.substitution' present in sys.modules 

'launch.launch_context' present in sys.modules 

'launch.condition' present in sys.modules 

'launch.action' present in sys.modules 

'launch.frontend.entity' present in sys.modules 

'launch.frontend.type_utils' present in sys.modules 

'launch.some_substitutions_type' present in sys.modules 

'launch.frontend.expose' present in sys.modules 

'ament_index_python.constants' present in sys.modules 

'pathlib' present in sys.modules 

'ament_index_python.search_paths' present in sys.modules 

'ament_index_python.resources' present in sys.modules 

'ament_index_python.packages' present in sys.modules 

'ament_index_python' present in sys.modules 

'lark.tree' present in sys.modules 

'_ast' present in sys.modules 

'ast' present in sys.modules 

'lark.utils' present in sys.modules 

'lark.exceptions' present in sys.modules 

'lark.lexer' present in sys.modules 

'lark.visitors' present in sys.modules 

'lark.parse_tree_builder' present in sys.modules 

'lark.parsers' present in sys.modules 

'lark.grammar' present in sys.modules 

'lark.parsers.grammar_analysis' present in sys.modules 

'lark.parsers.earley_common' present in sys.modules 

'lark.parsers.earley_forest' present in sys.modules 

'lark.parsers.earley' present in sys.modules 

'lark.parsers.xearley' present in sys.modules 

'lark.parsers.cyk' present in sys.modules 

'lark.parsers.lalr_analysis' present in sys.modules 

'lark.parsers.lalr_parser' present in sys.modules 

'lark.common' present in sys.modules 

'lark.parser_frontends' present in sys.modules 

'lark.load_grammar' present in sys.modules 

'lark.lark' present in sys.modules 

'lark' present in sys.modules 

'launch.substitutions.anon_name' present in sys.modules 

'shlex' present in sys.modules 

'launch.substitutions.substitution_failure' present in sys.modules 

'launch.substitutions.command' present in sys.modules 

'launch.substitutions.environment_variable' present in sys.modules 

'termios' present in sys.modules 

'tty' present in sys.modules 

'pty' present in sys.modules 

'osrf_pycommon.process_utils.get_loop_impl' present in sys.modules 

'osrf_pycommon.process_utils.async_execute_process_asyncio.impl' present in sys.modules 

'osrf_pycommon.process_utils.async_execute_process_asyncio' present in sys.modules 

'osrf_pycommon.process_utils.async_execute_process' present in sys.modules 

'osrf_pycommon.process_utils.execute_process_nopty' present in sys.modules 

'osrf_pycommon.process_utils.execute_process_pty' present in sys.modules 

'osrf_pycommon.process_utils.impl' present in sys.modules 

'osrf_pycommon.process_utils' present in sys.modules 

'launch.substitutions.find_executable' present in sys.modules 

'launch.substitutions.launch_configuration' present in sys.modules 

'launch.utilities.class_tools_impl' present in sys.modules 

'launch.utilities.create_future_impl' present in sys.modules 

'launch.utilities.ensure_argument_type_impl' present in sys.modules 

'launch.utilities.normalize_to_list_of_substitutions_impl' present in sys.modules 

'launch.utilities.perform_substitutions_impl' present in sys.modules 

'launch.utilities.signal_management' present in sys.modules 

'launch.utilities.visit_all_entities_and_collect_futures_impl' present in sys.modules 

'launch.utilities' present in sys.modules 

'launch.substitutions.local_substitution' present in sys.modules 

'launch.substitutions.path_join_substitution' present in sys.modules 

'launch.substitutions.python_expression' present in sys.modules 

'launch.substitutions.text_substitution' present in sys.modules 

'launch.substitutions.this_launch_file' present in sys.modules 

'launch.substitutions.this_launch_file_dir' present in sys.modules 

'launch.substitutions' present in sys.modules 

'launch.frontend.parse_substitution' present in sys.modules 

'launch.invalid_launch_file_error' present in sys.modules 

'launch.frontend.parser' present in sys.modules 

'launch.frontend' present in sys.modules 

'launch.actions.declare_launch_argument' present in sys.modules 

'launch.actions.emit_event' present in sys.modules 

'launch.actions.opaque_function' present in sys.modules 

'launch.events.process.running_process_event' present in sys.modules 

'launch.events.process.process_exited' present in sys.modules 

'launch.events.process.process_io' present in sys.modules 

'launch.events.process.process_matchers' present in sys.modules 

'launch.events.process.process_started' present in sys.modules 

'launch.events.process.process_stderr' present in sys.modules 

'launch.events.process.process_stdin' present in sys.modules 

'launch.events.process.process_stdout' present in sys.modules 

'launch.events.process.process_targeted_event' present in sys.modules 

'launch.events.process.shutdown_process' present in sys.modules 

'launch.events.process.signal_process' present in sys.modules 

'launch.events.process' present in sys.modules 

'launch.events.execution_complete' present in sys.modules 

'launch.launch_description' present in sys.modules 

'launch.events.include_launch_description' present in sys.modules 

'launch.events.matchers' present in sys.modules 

'launch.events.shutdown' present in sys.modules 

'launch.events.timer_event' present in sys.modules 

'launch.events' present in sys.modules 

'launch.actions.timer_action' present in sys.modules 

'launch.conditions.invalid_condition_expression_error' present in sys.modules 

'launch.conditions.evaluate_condition_expression_impl' present in sys.modules 

'launch.conditions.if_condition' present in sys.modules 

'launch.conditions.launch_configuration_equals' present in sys.modules 

'launch.conditions.launch_configuration_not_equals' present in sys.modules 

'launch.conditions.unless_condition' present in sys.modules 

'launch.conditions' present in sys.modules 

'launch.event_handlers.event_named' present in sys.modules 

'launch.event_handlers.on_execution_complete' present in sys.modules 

'launch.event_handlers.on_include_launch_description' present in sys.modules 

'launch.event_handlers.on_process_exit' present in sys.modules 

'launch.event_handlers.on_process_io' present in sys.modules 

'launch.event_handlers.on_process_start' present in sys.modules 

'launch.event_handlers.on_shutdown' present in sys.modules 

'launch.event_handlers' present in sys.modules 

'launch.actions.execute_process' present in sys.modules 

'launch.actions.pop_launch_configurations' present in sys.modules 

'launch.actions.push_launch_configurations' present in sys.modules 

'launch.actions.set_launch_configuration' present in sys.modules 

'launch.actions.group_action' present in sys.modules 

'launch.launch_description_source' present in sys.modules 

'launch.launch_description_sources.frontend_launch_file_utilities' present in sys.modules 

'launch.launch_description_sources.python_launch_file_utilities' present in sys.modules 

'launch.launch_description_sources.any_launch_file_utilities' present in sys.modules 

'launch.launch_description_sources.any_launch_description_source' present in sys.modules 

'launch.launch_description_sources.frontend_launch_description_source' present in sys.modules 

'launch.launch_description_sources.python_launch_description_source' present in sys.modules 

'launch.launch_description_sources' present in sys.modules 

'launch.actions.include_launch_description' present in sys.modules 

'launch.actions.log_info' present in sys.modules 

'launch.actions.opaque_coroutine' present in sys.modules 

'launch.actions.register_event_handler' present in sys.modules 

'launch.actions.set_environment_variable' present in sys.modules 

'launch.actions.shutdown_action' present in sys.modules 

'launch.actions.unregister_event_handler' present in sys.modules 

'launch.actions.unset_environment_variable' present in sys.modules 

'launch.actions.unset_launch_configuration' present in sys.modules 

'launch.actions' present in sys.modules 

'launch.launch_introspector' present in sys.modules 

'launch.launch_service' present in sys.modules 

'launch' present in sys.modules 

'launch_testing.util.proc_lookup' present in sys.modules 

'launch_testing.util' present in sys.modules 

'launch_testing.asserts.assert_exit_codes' present in sys.modules 

'launch_testing.asserts.assert_output' present in sys.modules 

'launch_testing.asserts.assert_sequential_output' present in sys.modules 

'launch_testing.asserts' present in sys.modules 

'launch_testing.io_handler' present in sys.modules 

'launch_testing.proc_info_handler' present in sys.modules 

'launch_testing.tools.process' present in sys.modules 

'launch_testing.tools' present in sys.modules 

'launch_testing.decorator' present in sys.modules 

'launch_testing.parametrize' present in sys.modules 

'launch_testing.ready_aggregator' present in sys.modules 

'launch_testing' present in sys.modules 

'gettext' present in sys.modules 

'argparse' present in sys.modules 

'xml.etree' present in sys.modules 

'xml.etree.ElementPath' present in sys.modules 

'_elementtree' present in sys.modules 

'xml.etree.ElementTree' present in sys.modules 

'launch_testing.junitxml' present in sys.modules 

'unittest.util' present in sys.modules 

'unittest.result' present in sys.modules 

'difflib' present in sys.modules 

'unittest.case' present in sys.modules 

'unittest.async_case' present in sys.modules 

'unittest.suite' present in sys.modules 

'unittest.loader' present in sys.modules 

'unittest.signals' present in sys.modules 

'unittest.runner' present in sys.modules 

'unittest.main' present in sys.modules 

'unittest' present in sys.modules 

'launch_testing.actions.test' present in sys.modules 

'launch_testing.actions.gtest' present in sys.modules 

'launch_testing.actions.pytest' present in sys.modules 

'launch_testing.actions.ready' present in sys.modules 

'launch_testing.actions' present in sys.modules 

'launch_testing.loader' present in sys.modules 

'launch_testing.print_arguments' present in sys.modules 

'launch_testing.parse_arguments' present in sys.modules 

'launch_testing.test_result' present in sys.modules 

'launch_testing.test_runner' present in sys.modules 

'launch_testing.launch_test' present in sys.modules 

'_pytest._version' present in sys.modules 

'_pytest' present in sys.modules 

'_uuid' present in sys.modules 

'uuid' present in sys.modules 

'attr._config' present in sys.modules 

'attr._compat' present in sys.modules 

'attr.exceptions' present in sys.modules 

'attr._make' present in sys.modules 

'attr.converters' present in sys.modules 

'attr.filters' present in sys.modules 

'attr.validators' present in sys.modules 

'attr._funcs' present in sys.modules 

'attr._version_info' present in sys.modules 

'attr' present in sys.modules 

'_pytest.outcomes' present in sys.modules 

'_csv' present in sys.modules 

'csv' present in sys.modules 

'configparser' present in sys.modules 

'importlib.metadata' present in sys.modules 

'_pytest.compat' present in sys.modules 

'_pytest.warning_types' present in sys.modules 

'_pytest.deprecated' present in sys.modules 

'pytest.collect' present in sys.modules 

'py.error' present in sys.modules 

'py._error' present in sys.modules 

'py._vendored_packages' present in sys.modules 

'py._vendored_packages.apipkg.version' present in sys.modules 

'py._vendored_packages.apipkg' present in sys.modules 

'py._version' present in sys.modules 

'py.test' present in sys.modules 

'py.process' present in sys.modules 

'py.apipkg' present in sys.modules 

'py.iniconfig' present in sys.modules 

'py.path' present in sys.modules 

'py.code' present in sys.modules 

'py.builtin' present in sys.modules 

'py.io' present in sys.modules 

'py.xml' present in sys.modules 

'py.log' present in sys.modules 

'py' present in sys.modules 

'unicodedata' present in sys.modules 

'_pytest._io.wcwidth' present in sys.modules 

'_pytest._io.terminalwriter' present in sys.modules 

'_pytest._io' present in sys.modules 

'_pytest._io.saferepr' present in sys.modules 

'pluggy._version' present in sys.modules 

'pluggy._tracing' present in sys.modules 

'pluggy.callers' present in sys.modules 

'pluggy.hooks' present in sys.modules 

'pluggy.manager' present in sys.modules 

'pluggy' present in sys.modules 

'_pytest._code.source' present in sys.modules 

'py._path' present in sys.modules 

'py._path.common' present in sys.modules 

'py._path.local' present in sys.modules 

'_pytest._code.code' present in sys.modules 

'_pytest._code' present in sys.modules 

'_pytest.assertion.util' present in sys.modules 

'_pytest.hookspec' present in sys.modules 

'_pytest.config.exceptions' present in sys.modules 

'iniconfig' present in sys.modules 

'_pytest.pathlib' present in sys.modules 

'_pytest.config.findpaths' present in sys.modules 

'_pytest.store' present in sys.modules 

'_pytest.config' present in sys.modules 

'_pytest.mark.expression' present in sys.modules 

'_pytest.mark.structures' present in sys.modules 

'_pytest.config.argparsing' present in sys.modules 

'_pytest.mark' present in sys.modules 

'_pytest.nodes' present in sys.modules 

'_pytest.fixtures' present in sys.modules 

'_pytest.reports' present in sys.modules 

'bdb' present in sys.modules 

'_pytest.timing' present in sys.modules 

'_pytest.runner' present in sys.modules 

'_pytest.main' present in sys.modules 

'_pytest.assertion.rewrite' present in sys.modules 

'_pytest.assertion.truncate' present in sys.modules 

'_pytest.assertion' present in sys.modules 

'_json' present in sys.modules 

'json.scanner' present in sys.modules 

'json.decoder' present in sys.modules 

'json.encoder' present in sys.modules 

'json' present in sys.modules 

'_pytest.python' present in sys.modules 

'_pytest.cacheprovider' present in sys.modules 

'_pytest.capture' present in sys.modules 

'_pytest.debugging' present in sys.modules 

'_pytest.freeze_support' present in sys.modules 

'_pytest.terminal' present in sys.modules 

'_pytest.logging' present in sys.modules 

'_pytest.monkeypatch' present in sys.modules 

'gc' present in sys.modules 

'_pytest.tmpdir' present in sys.modules 

'_pytest.pytester' present in sys.modules 

'numbers' present in sys.modules 

'_decimal' present in sys.modules 

'decimal' present in sys.modules 

'_pytest.python_api' present in sys.modules 

'_pytest.recwarn' present in sys.modules 

'pytest' present in sys.modules 

'rclpy.context' present in sys.modules 

'array' present in sys.modules 

'rcl_interfaces' present in sys.modules 

'rosidl_parser' present in sys.modules 

'rosidl_parser.definition' present in sys.modules 

'rcl_interfaces.msg._floating_point_range' present in sys.modules 

'rcl_interfaces.msg._integer_range' present in sys.modules 

'rcl_interfaces.msg._list_parameters_result' present in sys.modules 

'rcl_interfaces.msg._log' present in sys.modules 

'rcl_interfaces.msg._parameter' present in sys.modules 

'rcl_interfaces.msg._parameter_descriptor' present in sys.modules 

'rcl_interfaces.msg._parameter_event' present in sys.modules 

'rcl_interfaces.msg._parameter_event_descriptors' present in sys.modules 

'rcl_interfaces.msg._parameter_type' present in sys.modules 

'rcl_interfaces.msg._parameter_value' present in sys.modules 

'rcl_interfaces.msg._set_parameters_result' present in sys.modules 

'rcl_interfaces.msg' present in sys.modules 

'rclpy.parameter' present in sys.modules 

'rclpy.task' present in sys.modules 

'rclpy.constants' present in sys.modules 

'rclpy.utilities' present in sys.modules 

'rclpy' present in sys.modules 

'cartographer_ros_msgs' present in sys.modules 

'cartographer_ros_msgs.msg._landmark_entry' present in sys.modules 

'cartographer_ros_msgs.msg._landmark_list' present in sys.modules 

'cartographer_ros_msgs.msg._sensor_topics' present in sys.modules 

'cartographer_ros_msgs.msg._status_code' present in sys.modules 

'cartographer_ros_msgs.msg._status_response' present in sys.modules 

'cartographer_ros_msgs.msg._submap_entry' present in sys.modules 

'cartographer_ros_msgs.msg._submap_list' present in sys.modules 

'cartographer_ros_msgs.msg._submap_texture' present in sys.modules 

'cartographer_ros_msgs.msg._trajectory_options' present in sys.modules 

'cartographer_ros_msgs.msg' present in sys.modules 

'geometry_msgs' present in sys.modules 

'geometry_msgs.msg._accel' present in sys.modules 

'geometry_msgs.msg._accel_stamped' present in sys.modules 

'numpy._globals' present in sys.modules 

'numpy.__config__' present in sys.modules 

'numpy.version' present in sys.modules 

'numpy._distributor_init' present in sys.modules 

'numpy.core.info' present in sys.modules 

'numpy.core._multiarray_umath' present in sys.modules 

'numpy.compat._inspect' present in sys.modules 

'numpy.compat.py3k' present in sys.modules 

'numpy.compat' present in sys.modules 

'numpy.core.overrides' present in sys.modules 

'numpy.core.multiarray' present in sys.modules 

'numpy.core.umath' present in sys.modules 

'numpy.core._string_helpers' present in sys.modules 

'numpy.core._dtype' present in sys.modules 

'numpy.core._type_aliases' present in sys.modules 

'numpy.core.numerictypes' present in sys.modules 

'numpy.core._exceptions' present in sys.modules 

'numpy.core._asarray' present in sys.modules 

'numpy.core._ufunc_config' present in sys.modules 

'numpy.core._methods' present in sys.modules 

'numpy.core.fromnumeric' present in sys.modules 

'numpy.core.arrayprint' present in sys.modules 

'numpy.core.numeric' present in sys.modules 

'numpy.core.defchararray' present in sys.modules 

'numpy.core.records' present in sys.modules 

'numpy.core.memmap' present in sys.modules 

'numpy.core.function_base' present in sys.modules 

'numpy.core.machar' present in sys.modules 

'numpy.core.getlimits' present in sys.modules 

'numpy.core.shape_base' present in sys.modules 

'numpy.core.einsumfunc' present in sys.modules 

'numpy.core._multiarray_tests' present in sys.modules 

'numpy.core._add_newdocs' present in sys.modules 

'_ctypes' present in sys.modules 

'ctypes._endian' present in sys.modules 

'ctypes' present in sys.modules 

'numpy.core._dtype_ctypes' present in sys.modules 

'numpy.core._internal' present in sys.modules 

'numpy._pytesttester' present in sys.modules 

'numpy.core' present in sys.modules 

'numpy.lib.info' present in sys.modules 

'numpy.lib.ufunclike' present in sys.modules 

'numpy.lib.type_check' present in sys.modules 

'numpy.linalg.info' present in sys.modules 

'numpy.lib.twodim_base' present in sys.modules 

'numpy.linalg.lapack_lite' present in sys.modules 

'numpy.linalg._umath_linalg' present in sys.modules 

'numpy.linalg.linalg' present in sys.modules 

'numpy.linalg' present in sys.modules 

'numpy.matrixlib.defmatrix' present in sys.modules 

'numpy.matrixlib' present in sys.modules 

'numpy.lib.histograms' present in sys.modules 

'numpy.lib.function_base' present in sys.modules 

'numpy.lib.stride_tricks' present in sys.modules 

'numpy.lib.index_tricks' present in sys.modules 

'numpy.lib.mixins' present in sys.modules 

'numpy.lib.nanfunctions' present in sys.modules 

'numpy.lib.shape_base' present in sys.modules 

'numpy.lib.scimath' present in sys.modules 

'numpy.lib.polynomial' present in sys.modules 

'numpy.lib.utils' present in sys.modules 

'numpy.lib.arraysetops' present in sys.modules 

'numpy.lib.format' present in sys.modules 

'numpy.lib._datasource' present in sys.modules 

'numpy.lib._iotools' present in sys.modules 

'numpy.lib.npyio' present in sys.modules 

'numpy.lib.financial' present in sys.modules 

'numpy.lib.arrayterator' present in sys.modules 

'numpy.lib.arraypad' present in sys.modules 

'numpy.lib._version' present in sys.modules 

'numpy.lib' present in sys.modules 

'numpy.fft._pocketfft_internal' present in sys.modules 

'numpy.fft._pocketfft' present in sys.modules 

'numpy.fft.helper' present in sys.modules 

'numpy.fft' present in sys.modules 

'numpy.polynomial.polyutils' present in sys.modules 

'numpy.polynomial._polybase' present in sys.modules 

'numpy.polynomial.polynomial' present in sys.modules 

'numpy.polynomial.chebyshev' present in sys.modules 

'numpy.polynomial.legendre' present in sys.modules 

'numpy.polynomial.hermite' present in sys.modules 

'numpy.polynomial.hermite_e' present in sys.modules 

'numpy.polynomial.laguerre' present in sys.modules 

'numpy.polynomial' present in sys.modules 

'cython_runtime' present in sys.modules 

'numpy.random.common' present in sys.modules 

'numpy.random.bounded_integers' present in sys.modules 

'_cython_0_29_14' present in sys.modules 

'_hashlib' present in sys.modules 

'_blake2' present in sys.modules 

'_sha3' present in sys.modules 

'hashlib' present in sys.modules 

'hmac' present in sys.modules 

'secrets' present in sys.modules 

'numpy.random.bit_generator' present in sys.modules 

'numpy.random.mt19937' present in sys.modules 

'numpy.random.mtrand' present in sys.modules 

'numpy.random.philox' present in sys.modules 

'numpy.random.pcg64' present in sys.modules 

'numpy.random.sfc64' present in sys.modules 

'numpy.random.generator' present in sys.modules 

'numpy.random._pickle' present in sys.modules 

'numpy.random' present in sys.modules 

'numpy.ctypeslib' present in sys.modules 

'numpy.ma.core' present in sys.modules 

'numpy.ma.extras' present in sys.modules 

'numpy.ma' present in sys.modules 

'numpy.testing._private' present in sys.modules 

'numpy.testing._private.utils' present in sys.modules 

'numpy.testing._private.decorators' present in sys.modules 

'numpy.testing._private.nosetester' present in sys.modules 

'numpy.testing' present in sys.modules 

'numpy' present in sys.modules 

'geometry_msgs.msg._accel_with_covariance' present in sys.modules 

'geometry_msgs.msg._accel_with_covariance_stamped' present in sys.modules 

'geometry_msgs.msg._inertia' present in sys.modules 

'geometry_msgs.msg._inertia_stamped' present in sys.modules 

'geometry_msgs.msg._point' present in sys.modules 

'geometry_msgs.msg._point32' present in sys.modules 

'geometry_msgs.msg._point_stamped' present in sys.modules 

'geometry_msgs.msg._polygon' present in sys.modules 

'geometry_msgs.msg._polygon_stamped' present in sys.modules 

'geometry_msgs.msg._pose' present in sys.modules 

'geometry_msgs.msg._pose2_d' present in sys.modules 

'geometry_msgs.msg._pose_array' present in sys.modules 

'geometry_msgs.msg._pose_stamped' present in sys.modules 

'geometry_msgs.msg._pose_with_covariance' present in sys.modules 

'geometry_msgs.msg._pose_with_covariance_stamped' present in sys.modules 

'geometry_msgs.msg._quaternion' present in sys.modules 

'geometry_msgs.msg._quaternion_stamped' present in sys.modules 

'geometry_msgs.msg._transform' present in sys.modules 

'geometry_msgs.msg._transform_stamped' present in sys.modules 

'geometry_msgs.msg._twist' present in sys.modules 

'geometry_msgs.msg._twist_stamped' present in sys.modules 

'geometry_msgs.msg._twist_with_covariance' present in sys.modules 

'geometry_msgs.msg._twist_with_covariance_stamped' present in sys.modules 

'geometry_msgs.msg._vector3' present in sys.modules 

'geometry_msgs.msg._vector3_stamped' present in sys.modules 

'geometry_msgs.msg._wrench' present in sys.modules 

'geometry_msgs.msg._wrench_stamped' present in sys.modules 

'geometry_msgs.msg' present in sys.modules 

'nav_msgs' present in sys.modules 

'nav_msgs.msg._grid_cells' present in sys.modules 

'nav_msgs.msg._map_meta_data' present in sys.modules 

'nav_msgs.msg._occupancy_grid' present in sys.modules 

'nav_msgs.msg._odometry' present in sys.modules 

'nav_msgs.msg._path' present in sys.modules 

'nav_msgs.msg' present in sys.modules 

'webots_ros2_tests' present in sys.modules 

'rosgraph_msgs' present in sys.modules 

'rosgraph_msgs.msg._clock' present in sys.modules 

'rosgraph_msgs.msg' present in sys.modules 

'webots_ros2_tests.utils' present in sys.modules 
