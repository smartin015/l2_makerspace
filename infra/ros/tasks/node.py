# from l2_msgs.srv import GetProject
#from l2_msgs.msg import ProjectsUpdate
from todoist.api import TodoistAPI

import os
import rclpy
from rclpy.node import Node

class Server(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self):
        super().__init__('server')
        self.get_logger().info("Init")
        self.get_logger().info("Setting up service")
        self.setup_todoist()
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.ticks = 0

    def setup_todoist(self):
        self.todoist = TodoistAPI(os.getenv("TODOIST_API_TOKEN"))
        self.root_project_id = int(os.getenv("TODOIST_ROOT_PROJECT_ID") or "-1") 
        self.get_logger().info("Root project id: %d" % self.root_project_id)

    def timer_callback(self):
    if self.ticks % 5 == 0:
        self.publish_tasks_full()
        else:
            self.publish_tasks_delta()
    self.ticks++

    def publish_tasks_delta(self):
        # TODO diff against last_publish, then publish
    # TODO Replace with push updates 
    self.get_logger().info("publishing delta")
        # delta = ProjectsUpdate()
        self.todoist.sync()
        # self.pub.publish(delta)

    def publish_tasks_full(self):
        # TODO get all details and publish
    self.get_logger().info("publishing full")
        # full = ProjectsUpdate()
        # self.todoist.sync()
        print(self.todoist.state)
        # print([item['content'] for item in self.todoist.state['items'] if item['project_id'] == self.root_project_id])
        self.todoist.sync()
        # self.pub.publish(full)

def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
