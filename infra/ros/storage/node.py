from l2_msgs.msg import Object3D
import psycopg2
from pymongo import MongoClient
import os
import time
from watchdog.observers import Observer

import rclpy
from rclpy.node import Node

from l2_storage.file_sync import WatchHandler
from l2_storage.handlers import StorageHandlers
from l2_storage.tasks_sync import TaskSyncHandler
from l2_storage.gen_schema import gen_schema, get_table_msgs
from l2_storage.mongo import serialize, deserialize

class DBServer(Node):
    SDF_PUBLISH_PD = 10.0

    def __init__(self, ns="/l2/storage"):
        super().__init__('db_server', namespace=ns)
        self.get_logger().info("Init")
        self.connect_to_db()
        self.connect_to_mongo()
        self.sync_table_schema()

        self.storage_handlers = StorageHandlers(self)
        self.task_sync_handler = TaskSyncHandler(self)

        self.dirpath = self.get_parameter_or('dir_path', '/volume')
        self.watch_handler = WatchHandler(handler=self.upsert_object3d, dirpath=self.dirpath)
        self.observer = Observer()
        self.observer.schedule(self.watch_handler, self.dirpath, recursive=True)
        self.observer.start()

    def write_response(self, code, msg, response):
        response.status = code
        response.message = msg
        self.get_logger().info("%s: %s" % (code, msg))
        return response

    def create_upsert_query(self, table, pkey, cols):
        columns = ', '.join([f'{col}' for col in cols])
        constraint = ', '.join([f'{col}' for col in pkey])
        placeholder = ', '.join([f'%({col})s' for col in cols])
        updates = ', '.join([f'{col} = EXCLUDED.{col}' for col in cols])
        query = f"""INSERT INTO {table} ({columns}) 
              VALUES ({placeholder}) 
              ON CONFLICT ({constraint}) 
              DO UPDATE SET {updates};"""
        query.split()
        query = ' '.join(query.split())
        return query

    def upsert(self, table, kv):
        cursor = self.con.commit()
        cursor.execute(self.create_upsert_query(
            table=table,
            pkey=["id"],
            cols=kv.keys()), kv)
        self.con.commit(cursor)

    def lookup_object3d(self, name):
        cur = self.con.cursor()
        cur.execute("SELECT id FROM object3d WHERE name=%(name)s LIMIT 1", {"name": name})
        row = cur.fetchone()
        if row is None:
            self.get_logger().info("Object3D with name %s not found" % name)
            return None
        return row[0]


    def upsert_object3d(self, name, typestr, data):
        objtype = {
            ".wbt": Object3D.TYPE_PROTO,
            ".sdf": Object3D.TYPE_SDF,
            ".obj": Object3D.TYPE_OBJ,
        }.get(typestr, Object3D.TYPE_UNKNOWN)

        if objtype == Object3D.TYPE_UNKNOWN:
            return

        objid = self.lookup_object3d(name)
        data = {
            "type": int(objtype),
            "name": name,
            "data": data,
        }
        if objid is not None:
            data['id'] = objid
        cursor = self.con.cursor()
        cursor.execute(self.create_upsert_query(
                table="object3d",
                pkey=["id"],
                cols=["type", "name", "data"]),
                data)
        self.con.commit()
        cursor.close()
        self.get_logger().info("Added %s%s to db" % (name, typestr))

    def connect_to_mongo(self):
        # TODO configurable
        self.mongo = MongoClient('mongodb://app_user:password@mongo:27017')
        self.mongo.l2.object3d.insert_one(serialize(Object3D(name='testproj', id=5)))
        print(deserialize(self.mongo.l2.object3d.find_one({"name": "testproj"}), Object3D))

    def connect_to_db(self):
        from urllib.parse import urlparse
        result = urlparse(os.environ["PGRST_DB_URI"])
        username = result.username
        password = result.password
        database = result.path[1:]
        hostname = result.hostname
        
        while True:
            try:
                self.con = psycopg2.connect(
                            database=database, 
                            user=username, 
                            password=password, 
                            host=hostname)
                break
            except:
                self.get_logger().warn("Failed to connect to db; waiting a bit then retrying...")
                time.sleep(5)
        self.get_logger().info("Connected (host: %s, db: %s)" % (hostname, database))

    def sync_table_schema(self):
        expected = set([k.lower() for k in get_table_msgs().keys()])
        cur = self.con.cursor()
        cur.execute("""SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            ORDER BY table_name;""")
        found = set([r[0] for r in cur.fetchall()])
        cur.close()
        missing = expected - found
        if len(missing) == 0:
            self.get_logger().info("All expected tables present")
            return

        self.get_logger().warn("Creating missing expected tables: %s" % str(missing))
        cur = self.con.cursor()
        cur.execute(gen_schema(missing))
        print(cur.statusmessage)
        cur.close()

        # Ensure we have a root/default user
        cur = self.con.cursor()
        cur.execute("INSERT INTO l2actor (id, name) VALUES (1, 'root')")
        cur.close()

def main(args=None):
    rclpy.init(args=args)
    db_server = DBServer()
    rclpy.spin(db_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
