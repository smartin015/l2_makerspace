# from l2_msgs.srv import GetProject

import rclpy
from rclpy.node import Node

class Server(Node):

    def __init__(self):
        super().__init__('server')
        self.get_logger().info("Init")
        self.get_logger().info("Setting up service")
        #self.srv = self.create_service(GetProject, 'get_project', self.get_project_callback)
        
    #def get_project_callback(self, request, response):
    #    response.project.name = "test"
    #    self.get_logger().info('Incoming request\nname: %s' % (request.name))
    #
    #    return response

def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
