# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
import sys

# Snapshot from running freecad, opening python console, then 
# `import sys; print(sys.path)`
for p in ['/usr/share/freecad/Mod/Web', '/usr/share/freecad/Mod/Tux', '/usr/share/freecad/Mod/Test', '/usr/share/freecad/Mod/TechDraw', '/usr/share/freecad/Mod/Surface', '/usr/share/freecad/Mod/Start', '/usr/share/freecad/Mod/Spreadsheet', '/usr/share/freecad/Mod/Sketcher', '/usr/share/freecad/Mod/Show', '/usr/share/freecad/Mod/Ship', '/usr/share/freecad/Mod/Robot', '/usr/share/freecad/Mod/ReverseEngineering', '/usr/share/freecad/Mod/Raytracing', '/usr/share/freecad/Mod/Points', '/usr/share/freecad/Mod/Plot', '/usr/share/freecad/Mod/Path', '/usr/share/freecad/Mod/PartDesign', '/usr/share/freecad/Mod/Part', '/usr/share/freecad/Mod/OpenSCAD', '/usr/share/freecad/Mod/MeshPart', '/usr/share/freecad/Mod/Mesh', '/usr/share/freecad/Mod/Measure', '/usr/share/freecad/Mod/Material', '/usr/share/freecad/Mod/Inspection', '/usr/share/freecad/Mod/Import', '/usr/share/freecad/Mod/Image', '/usr/share/freecad/Mod/Idf', '/usr/share/freecad/Mod/Fem', '/usr/share/freecad/Mod/Drawing', '/usr/share/freecad/Mod/Draft', '/usr/share/freecad/Mod/Complete', '/usr/share/freecad/Mod/Arch', '/usr/share/freecad/Mod/AddonManager', '/usr/share/freecad/Mod', '/usr/lib/freecad/lib64', '/usr/lib/freecad-python3/lib', '/usr/share/freecad/Ext', '', '/usr/lib/python38.zip', '/usr/lib/python3.8', '/usr/lib/python3.8/lib-dynload', '/usr/local/lib/python3.8/dist-packages', '/usr/lib/python3/dist-packages', '', '/usr/lib/freecad/Macro']:
    sys.path.append(p)

import FreeCAD

class Example(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self):
        super().__init__('l2_example')
        self.get_logger().info("Init")
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("TODO")
        pass #todo self.pub.publish()

def main(args=None):
    rclpy.init(args=args)
    server = Example()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
