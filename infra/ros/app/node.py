# from l2_msgs.srv import GetProject
import os
import websockets
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
import http.server
import socketserver
import threading
import json
import asyncio
from random import choice
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Task
from l2_msgs.msg import Projects, Project, ProjectItem
from l2_msgs.srv import L2SearchProjects
from l2_msgs_exec.dict import to_dict

PORT = 8000
WS_PORT = 8001

class App(Node):
    def __init__(self, ns='l2'):
        super().__init__('l2_app', namespace=ns)
        self.get_logger().info("Init")
        os.chdir('/src')
        # self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        #self.pub = self.create_publisher(ProjectsUpdate, 'project', 10)
        self.estop_pub = self.create_publisher(Bool, "emergency_stop", 10)
        # self.active_project_cli = self.create_client(L2ActiveProjectSrv, "set_active_project")
        self.create_subscription(Projects, "projects", self.handle_projects, qos_profile=qos_profile_sensor_data) 

        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self.handle_joint_states, qos_profile=qos_profile_sensor_data)

        self.trajectory = None
        self.create_subscription(JointTrajectory, "/joint_trajectory_command", self.handle_joint_trajectory, qos_profile=qos_profile_sensor_data)
        self.fake_project_id = 0
        self.fake_task_id = 0
        self.projects = Projects(projects=[self.gen_fake_project() for i in range(4)])
        self.asyncs = []

    def set_wssrv(self, wssrv):
        self.wssrv = wssrv

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

    def handle_projects(self, msg):
        # Cache updates for future clients
        self.projects = msg
        # Send updates to all clients
        for ws in self.ws_clients:
            ws.send(to_dict(self.projects))

    def handle_joint_states(self, msg):
        if len(msg.name) == 0 or msg.name[0] != "joint_1":
            print("Skipping joints " + str(msg.name))
            return
        self.joint_state = msg
        self.asyncs.append(self.wssrv.broadcast_joint_states(self.joint_state, self.trajectory))

    def handle_joint_trajectory(self, msg):
        if len(msg.joint_names) == 0 or msg.joint_names[0] != "joint_1":
            print("Skipping trajectory " + str(msg.joint_names))
            return
        self.trajectory = msg
        self.asyncs.append(self.wssrv.broadcast_joint_states(self.joint_state, self.trajectory))

class WSServer:
    def __init__(self, node, loop, port):
        self.node = node
        self.loop = loop
        self.port = port
        self.jsq = asyncio.Queue(maxsize=1, loop=self.loop)
        self.ws_clients = set()
        self.joint_state_consumers = {}
        asyncio.ensure_future(self.jsq_handler())
        start_server = websockets.serve(self.on_app_ws, '', self.port, loop=self.loop)
        self.loop.run_until_complete(start_server)
        print("WS server init")

    async def broadcast_joint_states(self, msg):
        try:
          await self.jsq.put(msg)
        except Exception as e:
          self.node.get_logger().error(str(type(e)) + ":" + str(e))

   
    async def jsq_handler(self):
        self.node.get_logger().info("WS producer init")
        while True:
          msg = await self.jsq.get()
          data = {
            "pos": [int(i * 180 / 3.14159) for i in msg.position], # TODO stop using degrees in ui
            "target": [0]*len(msg.position),
            "limit": [0]*len(msg.position),
          }
          for ws in self.joint_state_consumers.values():
              await ws.send(json.dumps(data))
          self.jsq.task_done()

    async def on_app_ws(self, ws, path):
        print("New ws client")
        self.ws_clients.add(ws)
        try:
            await self.consumer_handler(ws, path)
        except Exception as e:
            self.node.get_logger().error(str(e))
        finally:
            self.ws_clients.remove(ws)

    async def consumer_handler(self, ws, path):
        # TODO callback into ROS
        async for msg in ws:
            if msg == "STOP":
                self.estop_pub.publish(Bool(data=True))
                await asyncio.gather([c.send("STOPPING") for c in self.ws_clients])
            elif msg == "JOINT_STATE":
                print("WS client %s registered for joint state updates" % ws.remote_address[0])
                self.joint_state_consumers[ws.remote_address[0]] = ws
            elif msg == "PROJECTS?":
                pd = to_dict(self.projects)
                pd["l2app"] = "project_list"
                await ws.send(json.dumps(pd))
            else:
                try:
                    json.loads(msg)
                    if msg.l2app == "active_project":
                        #cmd = L2ActiveProjectSrv(method="PUT", 
                        #    active_projects=L2ActiveProjects(actor_id=[1], project_id=[msg.id]))
                        #self.active_project_cli.call(cmd)
                        pass
                except Exception as e:
                    self.node.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    server = App()

    loop = asyncio.get_event_loop()

    # Initialize HTTP server
    httpd = socketserver.TCPServer(('', PORT), http.server.SimpleHTTPRequestHandler)
    httpd_thread = threading.Thread(target=httpd.serve_forever, daemon=True).start()
    server.get_logger().info("Serving HTTP at port %d" % PORT)
    
    # Initialize websocket server
    wssrv = WSServer(server, loop, WS_PORT)
    server.set_wssrv(wssrv)
    server.get_logger().info("Serving websockets at port %d" % WS_PORT)

    async def ros_spin_async():
      # executor = rclpy.executors.MultiThreadedExecutor()
      while True:
        # Spin ros, but don't block to wait on work
        rclpy.spin_once(server, timeout_sec=0.0)
        if len(server.asyncs) > 0:
            a = server.asyncs
            server.asyncs = []
            await asyncio.gather(*a, loop=loop)
    asyncio.ensure_future(ros_spin_async(), loop=loop)
    loop.set_debug(True)
    loop.run_forever()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
