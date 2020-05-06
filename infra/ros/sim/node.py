# from l2_msgs.srv import GetProject
import os
import rclpy
import asyncio
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from l2_msgs.msg import Simulation, Object3D
from l2_msgs.srv import GetObject3D
import uuid
import threading
from launch import LaunchDescription
from launch import LaunchService
import launch
import launch_ros.actions
import osrf_pycommon
from webots_ros2_core.utils import ControllerLauncher

class SimError(Exception):
    pass

class SimWorker(Node):
    PUBLISH_PD = 5  # seconds
    TEMP_PATH = "/tmp/world.wbt"

    def __init__(self, ls):
        name = 'l2_sim_worker_'+uuid.uuid4().hex
        super().__init__(name)
        self.ls = ls
        self.launch_ready = False
        self.declare_parameter('report_path', '', ParameterDescriptor())
        self.declare_parameter('object_config', '', ParameterDescriptor())
        self.get_logger().info("Init")
        self.sim_msg = Simulation(
            ns=self.get_namespace(),
            report_path=self.get_parameter('report_path').value,
            worker_id=name,
            object=Object3D(name=self.get_parameter('object_config').value))
        self.sim_pub = self.create_publisher(Simulation, 'simulation', 10)
        self.sim_pub_timer = self.create_timer(self.PUBLISH_PD, self.publish_sim)
        self.getcli = self.create_client(GetObject3D, '/l2/storage/get_object3d')
        self.get_logger().info("Waiting for storage")
        self.getcli.wait_for_service(timeout_sec=5)
        if not self.getcli.service_is_ready():
            raise SimError("Timed out waiting for storage")
        self.get_logger().info("GetObject3D %s" % self.sim_msg.object.name)
        future = self.getcli.call_async(GetObject3D.Request(name=self.sim_msg.object.name))
        future.add_done_callback(self.handle_response)

    def handle_response(self, response):
        if response.exception() is not None:
            raise response.exception()
        response = response.result()
        if not response.success or response.object.type != Object3D.TYPE_PROTO:
            raise SimError("Bad result: " + str(response))
        with open(self.TEMP_PATH, 'w') as f:
            f.write(response.object.data)
        self.get_logger().info("Response for %s written to %s" % (response.object.name, self.TEMP_PATH))
        self.launch()

    def launch(self):
    # Webots
        arguments = ['--mode=realtime', '--world='+self.TEMP_PATH, '--no-gui']
        webots = launch_ros.actions.Node(package='webots_ros2_core', node_executable='webots_launcher',
                                         arguments=arguments, output='screen')
        ld = LaunchDescription([
            webots,
            # Shutdown launch when Webots exits.
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ])
        self.ls.include_launch_description(ld)
        self.launch_ready = True
    # Note: launch service will be started from main thread
        self.publish_sim()

    def publish_sim(self):
        self.sim_pub.publish(self.sim_msg)

def main(args=None):
    rclpy.init(args=args)
    ls = LaunchService(debug=True)
    server = SimWorker(ls)
    print("Waiting until launch ready")
    while not server.launch_ready:
        rclpy.spin_once(server)
    loop = osrf_pycommon.process_utils.get_loop()
    launch_task = loop.create_task(ls.run_async())
    print("Spinning")
    # This is probably not the most awesome way to hand
    # off between rclpy and the launch service... but it works.
    # asyncio code copied from https://github.com/ros2/launch/pull/210
    while True:
        loop.run_until_complete(asyncio.sleep(0, loop=loop))
        rclpy.spin_once(server)
    loop.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
