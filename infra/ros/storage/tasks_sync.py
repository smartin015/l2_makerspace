from l2_msgs.msg import Projects
from rclpy.qos import qos_profile_sensor_data

class TaskSyncHandler:
    TOPIC="projects"

    def __init__(self, node):
        self.node = node
        node.create_subscription(Projects, TOPIC, self.handle_projects, qos_profile=qos_profile_sensor_data)
        self.get_logger().info("Subscribed to %s" % TOPIC)

    def get_logger(self):
        return self.node.get_logger()

    def handle_projects(self, msg):
        self.get_logger().info("Updating from projects message")
        for p in msg.projects:
            resp = self.node.storage_handlers.handlers['project'](L2Project(method="POST", project=p))
            self.get_logger().info(resp)
