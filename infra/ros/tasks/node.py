# from l2_msgs.srv import GetProject
import l2_msgs.msg as l2
import json
import re
from collections import defaultdict
from todoist.api import TodoistAPI

import rclpy
from rclpy.node import Node

class Server(Node):
    PUBLISH_PD = 60  # seconds

    def __init__(self, ns='l2'):
        self.name = 'tasks'
        super().__init__(self.name, namespace=ns)
        self.get_logger().info('Init')
        self.get_logger().info('Setting up service')
        with open('/config/todoist.json', 'r') as f:
            self.config = json.loads(f.read())
            self.get_logger().info('Config loaded')
        
        self.task_queue = []
        self.tasks_with_commands = set() # Note: not all tasks have valid commands
        self.pub = self.create_publisher(l2.Projects, 'projects', 10)
        self.timer = self.create_timer(self.PUBLISH_PD, self.timer_callback)
        self.ticks = 0

        # Init the todoist client and prime the system
        self.setup_todoist()
        self.timer_callback()

    def setup_todoist(self):
        self.todoist = TodoistAPI(self.config['TODOIST_API_TOKEN'])
        self.root_project_id = int(self.config['TODOIST_ROOT_PROJECT_ID'] or '-1') 
        self.get_logger().info('Root project id: %d' % self.root_project_id)

    def add_comment(self, task_id, content):
        self.todoist.notes.add(task_id, '[l2.%s]: %s' % (self.name, content))
        self.get_logger().info('Comment %s: %s' % (task_id, content))

    def extract_sequences(self, msg):
        for p in msg.projects:
            for i in p.items:
                fa = re.findall(r'`l2 (.*?)`', i.content)
                if len(fa) == 0 or i.id in self.tasks_with_commands:
                    continue
                
                self.tasks_with_commands.add(i.id)
                if len(fa) > 1:
                    self.add_comment(i.id, 'error: max of 1 command per task allowed; got %s' % str(fa))
                elif len(fa) == 1:
                    self.add_comment(i.id, 'adding to queue: %s'
                            % fa[0])
                    self.task_queue.append(fa[0])
                    
        self.todoist.commit() # Commit any comment changes

    def timer_callback(self):
        self.todoist.sync()

        # Get all subprojects of root project
        pmap = defaultdict(set)
        for p in self.todoist.state['projects']:
            pmap[str(p['parent_id'])].add(str(p['id']))
        traverse = [str(self.root_project_id)]
        projects = set()
        while len(traverse) > 0:
            p = traverse.pop(0)
            projects.add(p)
            for i in pmap[p]:
                if not i in projects:
                    traverse.append(i)
        print('Filtered projects: %s' % str(projects))
        projects = dict([(str(p['id']), l2.Project(name=p['name'], id=p['id'])) for p in
            self.todoist.state['projects'] if str(p['id']) in projects])
        nitems = 0
        for ti in self.todoist.state['items']:
            proj = projects.get(str(ti['project_id']))
            if proj is not None:
                nitems += 1
                item = l2.Item()
                item.id = ti['id']
                item.content = ti['content']
                proj.items.append(item)
        self.get_logger().info('Publishing %d projects with %d total items'
                % (len(projects), nitems))
        
        #self.todoist.sync()
        msg = l2.Projects(projects=list(projects.values()))
        self.pub.publish(msg)
        self.extract_sequences(msg)

def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
