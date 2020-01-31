from l2_msgs.srv import GetProject

import rclpy
from rclpy.node import Node

import psycopg2
import os

class DBServer(Node):

    def __init__(self):
        super().__init__('db_server')
        self.get_logger().info("Init")
        self.connect_to_db()
        self.get_logger().info("Setting up service")
        self.srv = self.create_service(GetProject, 'get_project', self.get_project_callback)
        
    def connect_to_db(self):
        self.get_logger().info("Parsing environment...")
        from urllib.parse import urlparse
        result = urlparse(os.environ["PGRST_DB_URI"])
        username = result.username
        password = result.password
        database = result.path[1:]
        hostname = result.hostname
        
        self.get_logger().info("Connecting to db...")
        self.con = psycopg2.connect(
                database=database, 
                user=username, 
                password=password, 
                host=hostname)
        self.get_logger().info("Connected!")

    def get_project_callback(self, request, response):
        response.project.name = "test"
        self.get_logger().info('Incoming request\nname: %s' % (request.name))

        return response

def main(args=None):
    rclpy.init(args=args)
    db_server = DBServer()
    rclpy.spin(db_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
