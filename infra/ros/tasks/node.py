# from l2_msgs.srv import GetProject
import l2_msgs.msg as l2
from l2_msgs.action import L2Sequence as L2SequenceAction
from diagnostic_msgs.msg import KeyValue
import json
import re
from collections import defaultdict
from todoist.api import TodoistAPI

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

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
        
        self.seq_queue = [] # Extracted sequences from tasks
        self.tasks_with_commands = set() # Note: not all tasks have valid commands
        self.pub = self.create_publisher(l2.Projects, 'projects', 10)
        self.seqcli = ActionClient(self, L2SequenceAction, 'sequence')
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

    def parse_sequence(self, uid, seqstr):
        seq = l2.L2Sequence(uid=uid)
        for part in seqstr.strip().split('|'):
            (name, params) = part.split(' ', 1)
            item = l2.L2SequenceItem(name=name)
            for param in params.split(','):
                (k,v) = param.split('=')
                item.params.append(KeyValue(key=k.strip(), value=v.strip()))
            seq.items.append(item)
        return seq

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
                    try:
                        seq = self.parse_sequence(str(i.id), fa[0])
                        self.add_comment(i.id, 'adding to queue: %s'
                            % str(seq))
                        self.seq_queue.append(seq)
                    except Exception as e:
                        self.add_comment(i.id, 'error: %s' % str(e))
                    
        self.todoist.commit() # Commit any comment changes
        if len(self.seq_queue) > 0:
            self.flush_work()

    def send_goal(self, client, goal, feedback_cb, done_cb):
        if not client.server_is_ready():
            self.get_logger().error('Action client not ready: %s' % client)
            return
        future = client.send_goal_async(goal,feedback_callback=feedback_cb)
        future.add_done_callback(done_cb)

    def flush_work(self):
        sq = self.seq_queue
        self.seq_queue = []
        self.get_logger().info("Sending %d actions" % len(sq))
        for seq in sq:
            self.send_goal(self.seqcli,
                L2SequenceAction.Goal(sequence=seq),
                self.handle_sequence_feedback,
                self.handle_sequence_accepted)

    def handle_sequence_feedback(self, msg):
        self.add_comment(int(msg.feedback.sequence.uid), "Feedback: %s" % msg.feedback)

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
        self.add_comment(response.sequence.uid, "result: %s" % response)

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
            if ti['checked'] == 1 or ti['is_deleted'] == 1:
                continue # Skip completed/deleted items
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
