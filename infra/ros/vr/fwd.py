# from l2_msgs.srv import GetProject
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist
import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

class TFFwd(Node):
    PUBLISH_PD = 0.1  # seconds
    SNAPSHOT_PD = 10.0 # seconds

    def __init__(self):
        super().__init__('l2_vr_fwd')
        self.get_logger().info("Init")
        self.sub = self.create_subscription(LinkStates, '/link_states', 
                self.sub_callback, qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(self.PUBLISH_PD, 
                self.timer_callback)
        self.pub = self.create_publisher(LinkStates, '/l2/vr/link_states', 
                qos_profile=qos_profile_sensor_data)
        self.links_received = {}
        self.links_published = {}
        self.next_snapshot = self.get_clock().now()


    def sub_callback(self, link_states):
        #for t in tf.transforms:
        #    self.tf[t.child_frame_id] = t
        self.links_received = {}
        for i in range(len(link_states.name)):
            # TODO include twist
            self.links_received[link_states.name[i]] = link_states.pose[i]

    def similar_tf(self, p1, q1, p2, q2, thresh = 0.01):
        return (abs(p1.x - p2.x) +
                abs(p1.y - p2.y) +
                abs(p1.z - p2.z) +
                abs(q1.x - q2.x) +
                abs(q1.y - q2.y) +
                abs(q1.z - q2.z) +
                abs(q1.w - q2.w)) < thresh

    def timer_callback(self):
        # Snapshotting updates new clients wwith the positions
        # of immobile objects and also clears out removed objects
        # from the "published" state.
        now = self.get_clock().now()
        if self.next_snapshot < now:
            self.links_published = {}
            self.next_snapshot = now + Duration(seconds=self.SNAPSHOT_PD)

        msg = LinkStates()
        for (k, v) in self.links_received.items():
            p = self.links_published.get(k, None)
            if not p or not self.similar_tf(
                    v.position, v.orientation, 
                    p.position, p.orientation):
                msg.name.append(k)
                msg.pose.append(v)
                msg.twist.append(Twist())
                self.links_published[k] = v
        if len(msg.name) > 0:
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
