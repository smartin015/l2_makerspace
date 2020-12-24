import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker

class AR3Jogger(Node):
    PUBLISH_PD = 0.05

    def __init__(self):
        super().__init__('ar3_jogger')
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.pub = self.create_publisher(TwistStamped, "servo_server/delta_twist_cmds", 10)
        self.marker_pub = self.create_publisher(Marker, "/delta_twist_marker", 10)
        self.i = 0

    def timer_callback(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        theta = (0.5 * self.get_clock().now().nanoseconds / 1000000000.0) % (2*3.14159265)
        m.twist.linear.x = 0.0
        m.twist.linear.z = math.sin(theta) * 0.3
        m.twist.linear.y = 0.0
        m.twist.angular.z = 0.0
        m.twist.angular.y = 0.0
        m.twist.angular.x = 0.0
        self.i = self.i + 1
        if self.i % 20 == 0:
            print(m.twist)
        self.pub.publish(m)

        # TODO servo to position, publish marker
        """
        mk = Marker()
        mk.header.stamp = m.header.stamp
        mk.header.frame_id = m.header.frame_id
        self.marker_pub.publish(Marker(
            header=m.header,
            ns='l2_ar3', 
            id=0,
            type=0,
            action=0,
            pose
        ))
        """
                
def main(args=None):
    rclpy.init(args=args)
    server = AR3Jogger()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
