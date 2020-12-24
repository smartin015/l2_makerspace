import threading
import time
import os
import rclpy
from rclpy.node import Node
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class MoveItDemo(Node):
    def __init__(self):
        super().__init__('moveit')

    def run(self):
        self.get_logger().info("Wawiting to let RViz initialize")
        time.sleep(5)
        self.get_logger().info("Initialzie MoveIt")
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rclpy.create_publisher('/move_group/display_planned_path', DisplayTrajectory, 20)
        print(group.get_planning_frame())
        print(group.get_end_effector_link())
        print(robot.get_group_names())
        print(robot.get_current_state())
#self.m.getPlanningSceneMonitor().providePlanningSceneService()
#self.m.getPlanningSceneMonitor().setPlanningScenePublishingFrequency(100)
#self.get_logger().info("Init PlanningComponent")
#self.arm = PlanningComponent("manipulator", self.m)
# Delay before running plan
#time.sleep(1)
# Create collision object
#obj = CollisionObject()
#obj.header.frame_id = "base_link"
#obj.id = "box"
#box = SolidPrimitive()
#box.type = box.BOX
#box.dimensions = (0.1, 0.4, 0.1)
#box_pose = Pose()
#box_pose.position.x = 0.4
#box_pose.position.z = 1.0
#obj.primitives.append(box)
#obj.primitive_poses.append(box_pose)
#obj.operation = obj.ADD
# Add to planning scene (may need to lock this)
# scene = self.m.getPlanningSceneMonitor().processCollisionObjectMsg(obj)
# Set goal
#self.get_logger().info("Set goal")
#self.arm.setGoal("extended")
#self.get_logger().info("Plan to goal")
#solution = self.arm.plan()
#if solution:
#    self.get_logger().info("Execute")
#    self.arm.execute()

def main(args=None):
    rclpy.init(args=args)
    demo = MoveItDemo()
    threading.Thread(target=demo.run, name='sim', daemon=True).start()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
