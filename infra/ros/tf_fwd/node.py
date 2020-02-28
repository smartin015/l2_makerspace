# from l2_msgs.srv import GetProject
from tf2_msgs.msg import TFMessage
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class TFFwd(Node):
    PUBLISH_PD = 0.1  # seconds

    def __init__(self):
        super().__init__('l2_tf_fwd')
        self.get_logger().info("Init l2_tf_fwd")
        self.sub = self.create_subscription(TFMessage, '/tf', 
                self.sub_callback, qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(self.PUBLISH_PD, 
                self.timer_callback)
        self.pub = self.create_publisher(TFMessage, '/tf_fwd', 
                qos_profile=qos_profile_sensor_data)
        self.tf = {}
        self.published = {}


    def sub_callback(self, tf):
        for t in tf.transforms:
            self.tf[t.child_frame_id] = t

    def should_publish(self, key):
        if self.published.get(key) is None:
            return True
        t = self.tf[key].transform
        p = self.published[key].transform
        return (abs(t.translation.x - p.translation.x) +
                abs(t.translation.y - p.translation.y) +
                abs(t.translation.z - p.translation.z) +
                abs(t.rotation.x - p.rotation.x) +
                abs(t.rotation.y - p.rotation.y) +
                abs(t.rotation.z - p.rotation.z) +
                abs(t.rotation.w - p.rotation.w)) > 0.01

    def timer_callback(self):
        msg = TFMessage()
        for k in self.tf.keys():
            if self.should_publish(k):
                msg.transforms.append(self.tf[k])
                self.published[k] = self.tf[k]

        if len(msg.transforms) > 0:
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    server = TFFwd()
    try:
        rclpy.spin(server)
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
