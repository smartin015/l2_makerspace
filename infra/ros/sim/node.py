# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from l2_msgs.msg import Simulation, Object3D, GetObject3D
import uuid
import threading
from launch import LaunchDescription
from launch import LaunchService
import launch_ros.actions

from webots_ros2_core.utils import ControllerLauncher

class SimWorker(Node):
    PUBLISH_PD = 60  # seconds
    TEMP_PATH = "/tmp/world.wbt"

    def __init__(self):
        name = 'l2_sim_worker_'+uuid.uuid4()
        super().__init__(name)

        self.sim_msg = Simulation(
            namespace=self.get_parameter('namespace'),
            report_path=self.get_parameter('report_path'),
            worker_id=name,
            object=Object3D(name=self.get_parameter('object_config')))
        self.get_logger().info("Init")
        self.sim_pub = self.create_publisher(Simulation, '', 10)
        self.sim_pub_timer = self.create_timer(self.PUBLISH_PD, self.publish_sim)
        self.publish_sim()
        self.resolve_and_launch()

    def resolve_and_launch(self):
        # TODO lookup from storage
        self.lookup_response_then_launch(GetObject3D.Response(
            success=True,
            message="ok",
            object=Object3D(
                type=Object3D.TYPE_PROTO,
                name="testobj",
                data="TODO",
                )
            ))

    def lookup_response_then_launch(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
        response = response.result()
        if not response.success or response.object.type != Object3D.TYPE_PROTO:
            self.logger().error("Bad result: " + str(response))
            return
        with open(self.TEMP_PATH, 'w') as f:
            f.write(response.object.data)
        print("Lookup response object written to temp file %s" % self.TEMP_PATH)
        self.launch()

    def launch(self):
	# Webots
        arguments = ['--mode=realtime', '--world='+self.TEMP_PATH, '--stdout', '--stderr']
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
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        self.sim_thread = threading.Thread(target=self.ls.run)
        self.get_logger().info("Launched simulator on new thread")
        self.sim_thread.start()

    def publish_sim(self):
        self.pub.publish(self.sim_msg)

def main(args=None):
    rclpy.init(args=args)
    server = SimWorker()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
