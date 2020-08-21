from l2_msgs.srv import L2Project, L2ProjectItem, Object3D
from l2_msgs.msg import Object3D
import psycopg2
import os
import time

import rclpy
from rclpy.node import Node

from l2_storage.file_sync import WatchHandler
from l2_storage.handlers import StorageHandlers
from l2_storage.gen_schema import gen_schema, get_table_msgs

class DBServer(Node):
    SDF_PUBLISH_PD = 10.0

    def __init__(self, ns="/l2/storage"):
        super().__init__('db_server', namespace=ns)
        self.get_logger().info("Init")
        self.connect_to_db()
        self.sync_table_schema()

        # Dirpath used for GetFile requests
        self.dirpath = self.get_parameter_or('dir_path', '/volume')
    
        self.handlers = StorageHandlers(self)
        self.create_service(L2Project, 'project', self.handlers.handle_project)
        self.create_service(L2ProjectItem, 'project_item', self.handlers.handle_project_item)
        self.create_service(Object3D, 'object3d', self.handlers.handle_object3d)

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


    def upsert_object3d(self, name, typestr, data):
        objtype = {
            ".wbt": Object3D.TYPE_PROTO,
            ".sdf": Object3D.TYPE_SDF,
            ".obj": Object3D.TYPE_OBJ,
        }.get(typestr, Object3D.TYPE_UNKNOWN)

        if objtype == Object3D.TYPE_UNKNOWN:
            return

        cursor = self.con.cursor()
        cursor.execute(self.create_upsert_query(
                table="object3d",
                pkey=["name"],
                cols=["objtype", "name", "data"],
            ), {
                  "objtype": int(objtype),
                  "name": name,
                  "data": data,
              })
        self.con.commit()
        cursor.close()
        self.get_logger().info("Added %s%s to db" % (name, typestr))

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
        expected = set(get_table_msgs(as_strings=True))
        cur = self.con.cursor()
        cur.execute("""SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            ORDER BY table_name;""")
        found = set([r[0] for r in cur.fetchall()])
        cur.close()
        missing = expected - found
        if len(missing) == 0:
            return

        self.get_logger().warn("Creating missing expected tables: %s" % str(missing))
        cur = self.con.cursor()
        cur.execute(gen_schema(missing))
        print(cur.fetchone())
        cur.close()

def main(args=None):
    rclpy.init(args=args)
    db_server = DBServer()
    rclpy.spin(db_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
