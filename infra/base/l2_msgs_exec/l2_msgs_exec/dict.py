# TODO: these are temporary until msg_type_to_cls can dynamically
# import message types
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from l2_msgs.msg import L2Actor, ProjectItem, Project, L2Role, Object3D
import re

def to_dict(msg):
    # Some raw data type
    if not hasattr(msg, 'get_fields_and_field_types'):
        return msg

    result = {}
    for n, t in msg.get_fields_and_field_types().items():
        if t.startswith('sequence'):
            result[n] = [to_dict(f) for f in getattr(msg, n)]
        else:
            v = getattr(msg, n)
            if hasattr(v, 'get_fields_and_field_types'):
                v = to_dict(v)
            result[n] = v

    return result

def msg_type_to_cls(pgtype):
    # Turns e.g. sequence<l2_msgs/L2Role> -> L2Role
    m = re.search(r"(\w+)\/(\w+)", pgtype)
    if m is None:
        return None
    # TODO: less awful way of doing this
    imp = 'from %s.msg import %s' % (m.group(1), m.group(2))
    print('TODO IMPORT', imp)
    # eval(imp)
    return eval(m.group(2))

def from_dict(obj, msgcls):
    # Some raw data type
    if msgcls is None:
        return obj
    result = msgcls()
    for n, t in msgcls.get_fields_and_field_types().items():
        typ = msg_type_to_cls(t)
        if t.startswith('sequence'):
            seq = [from_dict(i, typ) for i in obj[n]]
            setattr(result, n, seq)
        else:
            setattr(result, n, from_dict(obj[n], typ))
    return result

