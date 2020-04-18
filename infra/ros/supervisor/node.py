from l2_msgs.action import L2Sequence as L2SequenceAction
from l2_msgs.msg import L2Sequence, L2SequenceArray
import os
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import docker # https://docker-py.readthedocs.io/en/stable/
import yaml
from rclpy.node import Node

class Work():
    def __init__(self, goal_handle, supervisor):
        self.supervisor = supervisor
        self.goal_handle = goal_handle
        self.idx = 0
        self.sequence = self.goal_handle.request.sequence
        # https://github.com/ros2/ros2cli/blob/master/ros2action/ros2action/verb/send_goal.py#L123
        self.id = bytes(goal_handle.goal_id.uuid).hex()
        self.filters = {"label": "l2_work_id=" + self.id}
        self.info("Created")
    
    def info(self, text):
        self.supervisor.get_logger().info("%s: %s" % (self.id, text))

    def publish_feedback(self):
        self.goal_handle.publish_feedback(L2SequenceAction.Feedback(current_item=self.sequence.items[self.idx]))
    
    def stop_sequence(seq):
        self.info("Stopping")
        statuses = {}
        for c in self.supervisor.client.containers.list(filters=self.filters):
            if c.status in ["exited", "dead"]:
                continue
            # stop() blocks with grace period, then kills outright
            # wait() is how we get the status code.
            c.stop(timeout=self.WAIT_TIMEOUT)
            state = c.wait()
            if state["StatusCode"] != 0:
                seq.code = state["StatusCode"]
                # TODO also set status string from stderr
            self.info("%s (%s) stopped" % (c.name, c.id))
            statuses[c.status] = statuses.get(c.status, 0) + 1
        self.supervisor.client.containers.prune(filters=self.filters)
        self.info("Stopped, statuses %s" % statuses)

    def start_containers(self, item):
        container_ids = []
        for c in self.supervisor.config[item.name].items():
            image = c[1].get("image")
            if image is None:
                self.supervisor.get_logger().error("Config %s.%s has no image to run; ignoring" % (item.name, c[0]))
                continue
            cc = dict([(k, v) for (k, v) in c[1].items() if k in ["cmd", "volumes", "network"]])
            self.supervisor.client.containers.run(image, labels={"l2_work_id": self.id}, detach=True, pid_mode="host", **cc)
        return container_ids

    def active(self):
        return self.idx < len(self.sequence.items)

    def execute(self):
        while self.active():
            if not self.goal_handle.is_active:
                self.stop()
                return L2SequenceAction.Result(success=False, message="Aborted")
            elif self.goal_handle.is_cancel_requested:
                self.stop()
                self.goal_handle.canceled()
                return L2SequenceAction.Result(success=False, message="Canceled")

            # Get current container state, update sequence item timestamps
            # Per https://docs.docker.com/engine/reference/commandline/ps/, status is
            # created, restarting, running, removing, paused, exited, or dead
            running = False
            for c in self.supervisor.client.containers.list(filters=self.filters):
                # Paused containers aren't handled by the supervisor.
                if c.status == "paused":
                    self.supervisor.get_logger().warn("Container %s (%s) paused" % (c.name, c.short_id))
                    running = True
                elif c.status in ["created", "restarting", "running", "removing"]:
                    running = True
                else: # exited or dead
                   details = c.wait()
                   print(details)
                   if details["StatusCode"] != 0:
                     current.code = details["StatusCode"]
                     # TODO status string
                   c.remove()
                   self.info("%s (%s) removed (%s %d)" % (c.name, c.short_id, c.status, details["StatusCode"]))
        
            current = self.sequence.items[self.idx]
            if current.code != 0:
                self.stop_sequence(current)
                self.supervisor.get_logger().error
                self.goal_handle.abort()
                return L2SequenceAction.Result(success=False, message="Encountered failure in sequence item %s; aborting" % current.name)
            elif current.started.sec == 0:
                current.container_ids = self.start_containers(current)
                current.started = self.supervisor.get_clock().now().to_msg()
                self.info("%s started" % current.name)
            elif not running:
                self.info("%s completed" % current.name)
                current.stopped = self.supervisor.get_clock().now().to_msg()
                self.idx += 1
                if not self.active():
                    self.info("Succeeded")
                    self.goal_handle.succeed()
                    return L2SequenceAction.Result(success=True, message="L2Sequence Complete")

            self.publish_feedback()

class Supervisor(Node):
    PUBLISH_PD = 1  # seconds

    def __init__(self):
        super().__init__('l2_example')
        self.get_logger().info("Init")
        self.state_pub = self.create_publisher(L2SequenceArray, 'state', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.gc_state_callback)
        self.client = docker.from_env()
        self.config = {}
        self.state = []
        self.load_config()
        self._action_server = ActionServer(
            self,
            L2SequenceAction,
            'sequence',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()) 

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    def load_config(self):
        path = "/config.yaml"
        self.get_logger().info("Loading from %s" % path)
        with open(path, 'r') as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)
        self.get_logger().info(str(self.config))

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        for item in goal_request.sequence.items:
            if self.config.get(item.name) is None:
                self.get_logger().info('Rejecting incoming sequence; no config match for item %s' % item.name)
                return GoalResponse.REJECT
        self.get_logger().info('Accepting %d new sequence items' % len(goal_request.sequence.items))
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info('Accepting cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Executes the goal."""
        self.get_logger().info('Executing goal...')
        work = Work(goal_handle, self)
        self.state.append(work)
        return work.execute()

    def gc_state_callback(self):
        seqs = L2SequenceArray()
        for s in self.state:
            seqs.sequences.append(L2Sequence(items=s.sequence.items))
        self.state_pub.publish(seqs)
        # clean up state
        self.state = [s for s in self.state if s.active()]


def main(args=None):
    rclpy.init(args=args)
    server = Supervisor()
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
