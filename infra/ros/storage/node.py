from l2_msgs.srv import GetProject, GetFile, PutFile, GetObject3D
from l2_msgs.msg import Object3D
import time

import rclpy
from rclpy.node import Node

import psycopg2
import os

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class WatchHandler(FileSystemEventHandler):
    EXT_LIST = [".wbt", ".sdf"]

    def __init__(self, handler, dirpath):
        super().__init__()
        self.handler = handler
        # Push all files on init
        for dirName, subdirList, fileList in os.walk(dirpath):
            for fname in fileList:
                self._update(os.path.join(dirName, fname))

    def on_moved(self, event):
        self._update(event.dest_path)

    def on_created(self, event):
        if event.is_directory:
            return
        self._update(event.src_path)

    def on_modified(self, event):
        if event.is_directory:
            return
        self._update(event.src_path)

    def _update(self,path):
        (name, ext) = os.path.splitext(os.path.basename(path))
        if ext not in self.EXT_LIST:
            return
        with open(path, 'r') as f:
            data = f.read()
        self.handler(name, ext, data)


class DBServer(Node):
    SDF_PUBLISH_PD = 10.0

    def __init__(self, ns="/l2/storage"):
        super().__init__('db_server', namespace=ns)
        self.get_logger().info("Init")
        self.connect_to_db()

        # Dirpath used for GetFile requests
        self.dirpath = self.get_parameter_or('dir_path', '/volume')

        self.create_service(GetProject, 'get_project', self.get_project_callback)
        self.create_service(GetObject3D, 'get_object3d', self.get_object3d_callback)
        self.create_service(GetFile, 'get_file', self.get_file_callback)
        self.create_service(PutFile, 'put_file', self.put_file_callback)

        self.watch_handler = WatchHandler(handler=self.upsert, dirpath=self.dirpath)
        self.observer = Observer()
        self.observer.schedule(self.watch_handler, self.dirpath, recursive=True)
        self.observer.start()

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

    def upsert(self, name, typestr, data):
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
        self.get_logger().info("TODO: ensure initial tables are set up")

    def get_project_callback(self, request, response):
        self.get_logger().info(str(request))
        response.project.name = "test"
        self.get_logger().info(str(response))
        return response

    def get_object3d_callback(self, request, response):
        self.get_logger().info(str(request))
        cur = None
        try:
            cur = self.con.cursor()
            #cur.execute("SELECT object3d.objtype, object3d.data FROM object3d_registry " +
            #                        "LEFT JOIN object3d ON object3d.id=object3d_registry.object3d_id WHERE object3d_registry.name = %(name)s LIMIT 1", {"name": request.name})
            cur.execute("SELECT objtype, data FROM object3d WHERE name = %(name)s LIMIT 1", {"name": request.name})
            row = cur.fetchone()
            if row is None:
                response.success = False
                response.message = "Not found"
            else:
                response.object.type = row[0]
                response.object.data = row[1]
                response.object.name = request.name
                response.success = True
                response.message = "OK"
        except (Exception, psycopg2.Error) as error:
            response.success = False
            response.message = str(error)
        finally:
            if cur:
                cur.close()
        self.get_logger().info(str(response))
        return response

    def _resolve_path(self, path):
        return os.path.realpath(os.path.join(self.dirpath, path))

    def put_file_callback(self, request, response):
        resolved = self._resolve_path(request.path)
        self.get_logger().info("PutFile resolved: %s" % resolved)
        if not resolved.startswith(self.dirpath):
            response.success = False
            response.message = "Access denied: %s" % request.path
            self.get_logger().info(response.data)
            return response
        try:
            with open(resolved, 'w') as f:
                f.write(request.data)
                response.success = True
                self.get_logger().info('Wrote file (%dB): %s'
                        % (len(request.data), request.path))
        except:
            response.success = False
            response.message = "Could not write file"
        return response

    def get_file_callback(self, request, response):
        resolved = self._resolve_path(request.path)
        self.get_logger().info("Resolved: %s" % resolved)
        if not resolved.startswith(self.dirpath):
            response.success = False
            response.message = "Access denied: %s" % request.path
            self.get_logger().info(response.data)
            return response
        try:
            with open(resolved, 'r') as f:
                response.success = True
                response.data = f.read()
                self.get_logger().info('Read file (%dB): %s' % (len(response.data), request.path))
        except FileNotFoundError:
            response.success = False
            response.message = "Not found: %s" % request.path
            self.get_logger().info(response.data) 
        return response


def main(args=None):
    rclpy.init(args=args)
    db_server = DBServer()
    rclpy.spin(db_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
