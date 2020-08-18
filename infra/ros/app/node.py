# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
import http.server
import socketserver
import threading

class App(Node):
    PUBLISH_PD = 60  # seconds
    PORT = 8000

    def __init__(self):
        super().__init__('l2_app')
        self.get_logger().info("Init")
        self.handler = http.server.SimpleHTTPRequestHandler(directory='/src')
        self.httpd = socketserver.TCPServer(("", self.PORT), self.handler)
        self.get_logger().info("serving at port %d" % PORT)
        self.httpd_thread = threading.Thread(target=self.httpd.serve_forever,
                daemon=true).start()
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("TODO")
        pass #todo self.pub.publish()

def main(args=None):
    rclpy.init(args=args)
    server = App()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
