from l2_msgs.srv import GetProject, GetFile, GetObject3D

import rclpy
from rclpy.node import Node

import psycopg2
import os

class DBServer(Node):
  SDF_PUBLISH_PD = 10.0

  def __init__(self):
    super().__init__('db_server')
    self.get_logger().info("Init")
    self.connect_to_db()

    # Dirpath used for GetFile requests
    self.dirpath = self.get_parameter_or('dir_path', '/volume')

    self.create_service(GetProject, 'get_project', self.get_project_callback)
    self.create_service(GetObject3D, 'get_object3d', self.get_object3d_callback)
    self.create_service(GetFile, 'get_file', self.get_file_callback)
    self.get_logger().info("Services ready")
    
  def connect_to_db(self):
    self.get_logger().info("Parsing environment...")
    from urllib.parse import urlparse
    result = urlparse(os.environ["PGRST_DB_URI"])
    username = result.username
    password = result.password
    database = result.path[1:]
    hostname = result.hostname
    
    self.get_logger().info("Connecting to host %s, database %s" % (hostname, database))
    self.con = psycopg2.connect(
            database=database, 
            user=username, 
            password=password, 
            host=hostname)
    self.get_logger().info("Connected!")

  def get_project_callback(self, request, response):
    self.get_logger().info(str(request))
    response.project.name = "test"
    return response

  def get_object3d_callback(self, request, response):
    self.get_logger().info(str(request))
    response.object.name = "test"
    response.success = False
    response.message = "Unimplemented"
    return response

  def get_file_callback(self, request, response):
    resolved = os.path.realpath(os.path.join(self.dirpath, request.path))
    self.get_logger().info("Resolved: %s" % resolved)
    if not resolved.startswith(self.dirpath):
        response.success = False
        response.data = "Access denied: %s" % request.path
        self.get_logger().info(response.data)
        return response
    try:
        with open(resolved, 'r') as f:
            response.success = True
            response.data = f.read()
            self.get_logger().info('Read file (%dB): %s' % (len(response.data), request.path))
    except FileNotFoundError:
        response.success = False
        response.data = "Not found: %s" % request.path
        self.get_logger().info(response.data) 
    return response

def main(args=None):
  rclpy.init(args=args)
  db_server = DBServer()
  rclpy.spin(db_server)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
