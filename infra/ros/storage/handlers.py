from l2_msgs.srv import L2Project as L2ProjectSrv, L2ProjectItem as L2ProjectItemSrv, Object3D as Object3DSrv, L2Actor as L2ActorSrv
from l2_msgs.msg import Project, L2Actor, Object3D, ProjectItem
from l2_msgs.dict import to_dict, from_dict

class StorageHandlers:
    # Key is the field name within the *Srv message type
    # Value is [srv, msg]
    MSG_MAP = {
        'item': [L2ProjectItemSrv, ProjectItem],
        'object': [Object3DSrv, Object3D],
        'actor': [L2ActorSrv, L2Actor],
        'project': [L2ProjectSrv, Project],
    }

    def __init__(self, node):
        self.node = node
        self.handlers = {}
        for k, v in StorageHandlers.MSG_MAP.items():
            print("New handler %s for %s" % (k, str(v)))
            # TODO bind k and v within lambda
            
            self.handlers[k] = lambda request, response: self.handle_base(request, response, k, v[1])
            node.create_service(v[0], k, self.handlers[k])

    def get_logger(self):
        return self.node.get_logger()

    def write_response(self, code, msg, response):
        response.status = code
        response.message = msg
        self.get_logger().info("%s: %s" % (code, msg))
        return response

    def handle_base(self, request, response, field, cls):
        self.get_logger().info("Request: %s" % str(request))
        self.get_logger().info("Field %s, cls %s" % (str(field), str(cls)))
        tbl = getattr(self.node.mongo.l2, field)
        msg = getattr(request, field)
        if request.method == "POST":
            tbl.insert_one(to_dict(request.project))
            return self.write_response(200, 'ok', response)
        if request.method == "GET":
            if hasattr(msg, 'name') and msg.name != '':
                got = tbl.find_one({"name": msg.name})
            elif hasattr(msg, 'id'):
                got = tbl.find_one({"id": msg.id})
            if got is None:
                return self.write_response(404, 'not found', response)
            setattr(response, field, from_dict(got, cls))
            return self.write_response(200, 'ok', response)
        return self.write_response(403, 'invalid argument (method %s)' % request.method, response)

def main(args=None):
    rclpy.init(args=args)
    db_server = DBServer()
    rclpy.spin(db_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
