# This node handles ROS-side logic for the VR server
from l2_msgs.srv import GetFile, PutFile, SpawnObject3D, RemoveObject3D, GetObject3D
from l2_msgs.msg import Object3DArray, Object3D, Simulation, L2File, ROSState, L2Sequence as L2SequenceMsg
from l2_msgs.action import L2Sequence as L2SequenceAction
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from datetime import datetime

class TimeoutError(Exception):
    pass

class VRServer(Node):
    MAX_AGE = Duration(seconds=9.0)

    def __init__(self, ns="/l2"):
        super().__init__('l2_vr', namespace=ns)
        self.get_logger().info("Init")
        self.declare_parameter('extra_names', [], ParameterDescriptor())
        self.declare_parameter('verbose', False, ParameterDescriptor())
        T0 = Time(clock_type=self.get_clock().clock_type)

        self.verbose = self.get_parameter('verbose').value

        self.ignore_names = set(['ground_plane'])
        self.extra_names = self.get_parameter('extra_names').value
        if self.extra_names is None:
            self.extra_names = set()
        else:
            self.extra_names = set(self.extra_names)
        self.get_logger().info("Ignoring %s, appending %s" % (self.ignore_names, self.extra_names))
        self.sim_state = Simulation() # TODO support multiple simulations
        self.vr_object3d = Object3DArray()
        self.last_sim_msg = T0
        self.last_vr_msg = T0
        self.last_status = None
        
        # Node's default callback group is mutually exclusive. 
        # This would prevent the client response
        # from being processed until the timer callback finished,
        # but the timer callback in this
        # example is waiting for the client response
        cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.future_deadlines = []

        self.logtmr = self.create_timer(10.0, self.log_status, callback_group=cb_group)
        self.resolvetmr = self.create_timer(10.0, self.resolve_diffs, callback_group=cb_group)
        self.rosstatetmr = self.create_timer(10.0, self.publish_ros_state,
                callback_group=cb_group)
        self.getcli = self.create_client(GetObject3D, 'storage/get_object3d', callback_group=cb_group)
        self.putfilecli = self.create_client(PutFile, 'storage/put_file', callback_group=cb_group)
        self.spawncli = self.create_client(SpawnObject3D, 'vr/SpawnObject3D', callback_group=cb_group)
        self.rmcli = self.create_client(RemoveObject3D, 'vr/RemoveObject3D', callback_group=cb_group)
        self.seqcli = ActionClient(self, L2SequenceAction, 'sequence')
        self.vr_missing_pub = self.create_publisher(Object3DArray, "vr/missing_object3d", 10)
        self.vr_extra_pub = self.create_publisher(Object3DArray, "vr/extra_object3d", 10)
        self.ros_state_pub = self.create_publisher(ROSState, "vr/ros_state", 10)
        self.seq_state_pub = self.create_publisher(L2SequenceMsg,
            "vr/SequenceUpdate", 10)
        self.create_subscription(L2SequenceMsg, "vr/Sequence",
                self.handle_start_sequence, qos_profile_sensor_data,
                callback_group=cb_group)
        self.create_subscription(Object3DArray, "vr/Object3D", self.set_vr_state, qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(Simulation, "sim/simulation", self.set_sim_state, qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(L2File, "vr/PutFile", self.handle_put_file,
                qos_profile_sensor_data, callback_group=cb_group)

    def stale(self):
        now = self.get_clock().now()
        return (now - self.last_sim_msg > self.MAX_AGE) or (now - self.last_vr_msg > self.MAX_AGE)

    def sim_objset(self):
        if self.sim_state.object.name == "":
            return set()
        return set([self.sim_state.object.name]).difference(self.ignore_names)

    def vr_objset(self):
        return set([v.name for v in self.vr_object3d.objects])

    def send_goal(self, client, goal, feedback_cb, done_cb):
        if not client.server_is_ready():
            self.get_logger().error('Action client not ready: %s' % client)
            return
        future = client.send_goal_async(goal,feedback_callback=feedback_cb)
        future.add_done_callback(done_cb)
        #self.future_deadlines.append((future, self.get_clock().now()
        #    + Duration(seconds=seconds)))

    def call_with_deadline(self, client, req, callback, seconds=2):
        if not client.service_is_ready():
            self.get_logger().error('Not ready: %s' % client)
            return
        future = client.call_async(req)
        future.add_done_callback(callback)
        self.future_deadlines.append((future, self.get_clock().now() + Duration(seconds=seconds)))

    def publish_ros_state(self):
        nodes, node_ns = list(zip(*self.get_node_names_and_namespaces()))
        services, service_types = list(zip(*self.get_service_names_and_types()))
        topics, topic_types = list(zip(*self.get_topic_names_and_types()))
        self.ros_state_pub.publish(ROSState(
            topics = topics,
            topic_types = [t[0] for t in topic_types],
            services = services,
            service_types = [s[0] for s in service_types],
            nodes = nodes,
            node_namespaces = node_ns
        ))
        # self.get_logger().info("Published ROS state")


    def resolve_diffs(self):
        if self.stale():
            return

        # Look up any missing object configs via the storage ros node,
        # then push them to the VR server in the callback
        for name in self.sim_objset().difference(self.vr_objset()):
            self.get_logger().info("Lookup %s" % name)
            self.call_with_deadline(self.getcli, GetObject3D.Request(name=name), self.get_object3d_response)

        # Remove any objects on the VR server that are missing in sim
        for name in self.vr_objset().difference(self.sim_objset()):
            self.get_logger().info("Remove %s" % name)
            self.call_with_deadline(self.rmcli, RemoveObject3D.Request(name=name), self.log_response)

        self.log_status()

    def get_object3d_response(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
            return
        response = response.result()
        if not response.success:
            self.get_logger().warn("Bad result: " + str(response))
            return
        self.get_logger().info('Got response %s' % str(response))
        self.call_with_deadline(self.spawncli, SpawnObject3D.Request(object=response.object, scale=1.0), self.log_response)

    def log_response(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
            return
        self.get_logger().info(str(response.result()))
        
    def log_status(self):
        now = self.get_clock().now()
        status = {
            "sim": ["-%dsec" % (now-self.last_sim_msg).to_msg().sec, self.sim_objset()],
            "vr": ["-%dsec" % (now-self.last_vr_msg).to_msg().sec, self.vr_objset()],
            "stale": self.stale(),
        }

        # Skip if there's not really anything new to report
        if not self.verbose and self.last_status is not None and (
                self.last_status["stale"] == status["stale"] 
                and self.last_status["sim"][1] == status["sim"][1]
                and self.last_status["vr"][1] == status["vr"][1]):
            return
        self.get_logger().info(str(status))
        self.vr_missing_pub.publish(Object3DArray(objects=[Object3D(name=n) for n in self.vr_objset().difference(self.sim_objset())]))
        self.vr_extra_pub.publish(Object3DArray(objects=[Object3D(name=n) for n in self.sim_objset().difference(self.vr_objset())]))
        self.last_status = status

    def set_vr_state(self, msg):
        self.vr_object3d = msg
        self.last_vr_msg = self.get_clock().now()

    def set_sim_state(self, msg):
        self.sim_state = msg
        self.last_sim_msg = self.get_clock().now()

    def handle_start_sequence(self, msg):
        self.get_logger().info("Sequence %s" % msg)
        self.send_goal(self.seqcli,
                L2SequenceAction.Goal(sequence=msg),
                self.handle_sequence_feedback,
                self.handle_sequence_accepted)
    
    def handle_sequence_feedback(self, msg):
        self.get_logger().info("Feedback: %s" % msg.feedback)
        self.seq_state_pub.publish(msg.feedback.sequence)

    def handle_sequence_accepted(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
            return
        goal_handle = response.result() # Future<ClientGoalHandle>
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        fut = goal_handle.get_result_async()
        fut.add_done_callback(self.handle_sequence_result)

    def handle_sequence_result(self, response):
        response = response.result().result
        print("Sequence result: %s" % response)
        self.seq_state_pub.publish(response.sequence)

    def handle_put_file(self, msg):
        # Repackage and forward file writing request to 
        # storage
        self.get_logger().info("PutFile %s (%dB)" % (msg.path, len(msg.data)))
        self.call_with_deadline(self.putfilecli, PutFile.Request(path=msg.path,
            data=msg.data), self.put_file_response)

    def put_file_response(self, response):
        if response.exception() is not None:
            self.get_logger().error(str(response.exception()))
            return
        response = response.result()
        if not response.success:
            self.get_logger().warn("Bad result: " + str(response))
            return

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
