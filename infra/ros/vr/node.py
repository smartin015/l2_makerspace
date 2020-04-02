# This node handles ROS-side logic for the VR server
from l2_msgs.srv import GetFile, SpawnObject3D, GetObject3D
from l2_msgs.msg import Object3DArray, Object3D
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
import pprint
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

class TimeoutError(Exception):
    pass

class VRServer(Node):
    MAX_AGE = Duration(seconds=9.0)

    def __init__(self):
        super().__init__('l2_vr')
        T0 = Time(clock_type=self.get_clock().clock_type)
        self.ignore_names = set(['ground_plane'])
        self.sim_model_states = ModelStates()
        self.last_sim_msg = T0
        self.vr_object3d = Object3DArray()
        self.last_vr_msg = T0
        self.get_logger().info("Init")
        self.executor = rclpy.executors.MultiThreadedExecutor()
        # Node's default callback group is mutually exclusive. 
        # This would prevent the client response
        # from being processed until the timer callback finished,
        # but the timer callback in this
        # example is waiting for the client response
        cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.logtmr = self.create_timer(10.0, self.log_status, callback_group=cb_group)
        self.resolvetmr = self.create_timer(10.0, self.resolve_diffs, callback_group=cb_group)
        self.getcli = self.create_client(GetObject3D, '/get_object3d', callback_group=cb_group)
        self.spawncli = self.create_client(SpawnObject3D, '/l2/vr/SpawnObject3D', callback_group=cb_group)
        self.unresolved_pub = self.create_publisher(Object3DArray, "unresolved_object3d", 10)
        self.create_subscription(Object3DArray, "/l2/vr/Object3D", self.set_vr_state, qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(ModelStates, "/model_states", self.set_sim_state, qos_profile_sensor_data, callback_group=cb_group)
        self.future_deadlines = []

    def vr_missing(self):
        now = self.get_clock().now()
        # Return an empty diff if either sim or vr is stale
        if (now - self.last_sim_msg > self.MAX_AGE) or (now - self.last_vr_msg > self.MAX_AGE):
            return set()

        gz_objs = set(self.sim_model_states.name).difference(self.ignore_names)
        vr_objs = set([v.name for v in self.vr_object3d.objects])
        return gz_objs.difference(vr_objs)

    def resolve_diffs(self):
        for name in self.vr_missing():
            if not self.getcli.service_is_ready():
                self.get_logger().info('GetObject3D service not ready')
                continue
            req = GetObject3D.Request()
            req.name = name
            future = self.getcli.call_async(req)
            future.add_done_callback(self.get_object3d_response)
            self.future_deadlines.append((future, self.get_clock().now() + Duration(seconds=2)))

    def get_object3d_response(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
            return
        response = response.result()
        if not response.success:
            self.get_logger().warn("Bad result: " + str(response))
            return
        self.get_logger().info('Got response %s' % str(response))
        req = SpawnObject3D.Request()
        req.object = response.object
        req.scale = 1.0
        self.get_logger().info("Calling SpawnObject3D with object name %s" % req.object.name)
        future = self.spawncli.call_async(req)
        future.add_done_callback(self.spawn_object3d_response)
        self.future_deadlines.append((future, self.get_clock().now() + Duration(seconds=2)))

    def spawn_object3d_response(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
            return
        self.get_logger().info(str(response.result()))
        
    def log_status(self):
        status = {
            "sim_ts": self.last_sim_msg,
            "sim_objs": len(self.sim_model_states.name),
            "vr_ts": self.last_vr_msg,
            "vr_objs": len(self.vr_object3d.objects),
            "vr_missing": self.vr_missing(),
        }
        self.get_logger().info(pprint.pformat(status))
        result = Object3DArray()
        for u in status['vr_missing']:
            r = Object3D()
            r.name = u
            result.objects.append(r)
        self.unresolved_pub.publish(result)

    def set_vr_state(self, msg):
        self.vr_object3d = msg
        self.last_vr_msg = self.get_clock().now()

    def set_sim_state(self, msg):
        self.sim_model_states = msg
        self.last_sim_msg = self.get_clock().now()

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, executor=self.executor)
            incomplete = []
            for (f, d) in self.future_deadlines:
                if f.done():
                    continue 
                if self.get_clock().now() > d:
                    f.set_exception(TimeoutError("Deadline exceeded: %s" % d))
                    continue
                incomplete.append((f, d))
            self.future_deadlines = incomplete

def main(args=None):
    rclpy.init(args=args)
    vr_server = VRServer()
    vr_server.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
