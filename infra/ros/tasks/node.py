# from l2_msgs.srv import GetProject
import l2_msgs.msg as l2
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
        self.pub = self.create_publisher(l2.Projects, 'projects', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.ticks = 0
        self.timer_callback()

    def setup_todoist(self):
        self.todoist = TodoistAPI(os.getenv("TODOIST_API_TOKEN"))
        self.root_project_id = int(os.getenv("TODOIST_ROOT_PROJECT_ID") or "-1") 
        self.get_logger().info("Root project id: %d" % self.root_project_id)

    def timer_callback(self):
        # TODO get all details and publish
        self.get_logger().info("publishing full")
        # full = ProjectsUpdate()
        self.todoist.sync()
        proj = l2.Project()
        for ti in self.todoist.state['items']:
            if ti['project_id'] == self.root_project_id:
                item = l2.Item()
                item.id = ti['id']
                item.content = ti['content']
                proj.items.append(item)
        msg = l2.Projects()
        msg.projects.append(proj)
        print(msg)
        # print([item['content'] for item in self.todoist.state['items'] if item['project_id'] == self.root_project_id])
        #self.todoist.sync()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
