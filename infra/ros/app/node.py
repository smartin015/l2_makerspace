# from l2_msgs.srv import GetProject
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import http.server
import socketserver
import threading
import json
import asyncio
from random import choice
from rclpy.qos import qos_profile_sensor_data
from l2_msgs.msg import Projects, Project, ProjectItem, L2ActiveProject
from l2_msgs.srv import L2ActiveProject as L2ActiveProjectSrv
from l2_msgs_exec.dict import to_dict

class App(Node):
    PUBLISH_PD = 60  # seconds
    PORT = 8000

    def __init__(self):
        super().__init__('l2_app')
        self.get_logger().info("Init")
        self.handler = http.server.SimpleHTTPRequestHandler(directory='/src')
        self.httpd = socketserver.TCPServer(("", self.PORT), self.handler)
        self.get_logger().info("serving at port %d" % PORT)
        self.httpd_thread = threading.Thread(target=self.httpd.serve_forever,
                daemon=true).start()
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.estop_pub = self.create_publisher(Bool, "emergency_stop", 10)
        self.active_project_cli = self.create_client(L2ActiveProjectSrv, "set_active_project")
        self.create_subscription(Projects, "projects", self.handle_projects, qos_profile=qos_profile_sensor_data)  
        self.fake_project_id = 0
        self.fake_task_id = 0
        self.projects = Projects(projects=[self.gen_fake_project() for i in range(4)])

    def gen_fake_project(self):
        self.fake_project_id = self.fake_project_id+1
        adj = random.choice(["Project", "Soup", "VR", "Mobile", "Adjective"])
        noun = random.choice(["A", "B", "C", "Spiller", "Shooter", "Sim", "Beast", "Noun"])
        return Project(name="(Sample) %s %s" % (adj, noun), id=self.fake_project_id, 
            items=[self.gen_fake_task() for i in range(random.randint(3,5))])

    def gen_fake_task(self):
        self.fake_task_id = self.fake_task_id+1
        action = random.choice(["frob", "scrub", "burn", "save", "explode", "defrag", "prepare"])
        target = random.choice(["bandsaw", "drill press", "waterjet", "VR", "L2"])
        return ProjectItem(id=self.fake_task_id, content_type=ProjectItem.CONTENT_TYPE_L2_TASK, content="%s %s" % (action, target))

    async def consumer_handler(self, ws, path):
        async for msg in ws:
            if msg == "STOP":
                self.estop_pub.publish(Bool(data=True))
                await asyncio.gather([c.send("STOPPING") for c in self.ws_clients])
            elif msg == "PROJECTS?":
                pd = to_dict(self.projects)
                pd["l2app"] = "project_list"
                await ws.send(json.dumps(pd))
            else:
                try:
                    json.loads(msg)
                    if msg.l2app == "active_project":
                        self.active_project_cli.call(L2ActiveProjectSrv(method="PUT", 
                            active_projects=L2ActiveProject(actor_id=[1], project_id=[msg.id]))
                except Exception as e:
                    self.get_logger().error(str(e))

    async def on_app_ws(self, ws, path):
        self.ws_clients.add(ws)
        try:
            await self.consumer_handler(ws, path)
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self.ws_clients.remove(ws)

    def handle_projects(self, msg):
        # Cache updates for future clients
        self.projects = msg
        # Send updates to all clients
        for ws in self.ws_clients:
            ws.send(to_dict(self.projects))

def main(args=None):
    rclpy.init(args=args)
    server = App()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
