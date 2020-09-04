# from l2_msgs.srv import GetProject
import os
import websockets
import rclpy
from rclpy.node import Node
from std_msgs.msg import Boolean
import http.server
import socketserver
import threading
import json
from rclpy.qos import qos_profile_sensor_data
from l2_msgs.msg import Projects
from l2_msgs.dict import to_dict

class App(Node):
    PUBLISH_PD = 60  # seconds
    PORT = 8000
    WS_PORT = 8001
    ALLOWED_PUBLISH_TOPICS = [
        "/l2/app/emergency_stop"
    ]

    def __init__(self, ns='l2'):
        super().__init__('l2_app', namespace=ns)
        self.get_logger().info("Init")
        self.handler = http.server.SimpleHTTPRequestHandler(directory='/src')
        self.httpd = socketserver.TCPServer(('', self.PORT), self.handler)
        self.wss = websockets.Server(('', self.WS_PORT), self.on_app_message)
        self.get_logger().info("serving at port %d" % PORT)
        self.httpd_thread = threading.Thread(target=self.httpd.serve_forever,
                daemon=true).start()
        self.estop_pub = self.create_publisher(Boolean, "emergency_stop", 10)
        node.create_subscription(Projects, "projects", self.handle_projects, qos_profile=qos_profile_sensor_data)  

    def on_app_message(self, ws, msg):
        if msg == "STOP":
            self.estop_pub.publish(True)
            ws.write("STOPPING")

    def handle_projects(self, msg):
        # Send updates to all clients
        for ws in self.clients:
            ws.write(to_dict(msg))

def main(args=None):
    rclpy.init(args=args)
    server = App()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
