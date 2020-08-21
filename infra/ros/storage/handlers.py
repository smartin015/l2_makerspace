from l2_msgs.srv import L2Project, L2ProjectItem, Object3D
from l2_msgs.msg import Object3D

class StorageHandlers:
    def __init__(self, node, con):
        self.node = node

    def get_logger(self):
        return self.node.get_logger()

    def write_response(self, code, msg, response):
        response.status = code
        response.message = msg
        self.get_logger().info("%s: %s" % (code, msg))
        return response

    def handle_object3d(self, request, response):
        self.get_logger().info(str(request))
        cur = None
        try:
            cur = self.con.cursor()
            #cur.execute("SELECT object3d.objtype, object3d.data FROM object3d_registry " +
            #                        "LEFT JOIN object3d ON object3d.id=object3d_registry.object3d_id WHERE object3d_registry.name = %(name)s LIMIT 1", {"name": request.name})
            cur.execute("SELECT objtype, data FROM Object3D WHERE name = %(name)s LIMIT 1", {"name": request.name})
            row = cur.fetchone()
            if row is None:
                return self.write_response(404, "not found", response)
            else:
                response.object.type = row[0]
                response.object.data = row[1]
                response.object.name = request.name
                return self.write_response(200, "ok", response)
        except (Exception, psycopg2.Error) as error:
            return self.write_response(500, str(error), response)
        finally:
            if cur:
                cur.close()

    def handle_project(self, request, response):
        query = ["begin;"]
        # owner first (if more than just ID)
        if request.project.owner.name is not None:
            query.append("TODO INSERT")

        # then project
        query.append("TODO INSERT")

        # then all items (if any)
        query.append("commit;")

    def handle_project_item(self, request, response):
        fields = request.item.get_fields_and_field_types().keys()
        cursor = self.con.cursor()
        try:
            if request.method == "GET":
                cursor.execute("SELECT %s FROM ProjectItem WHERE id=%d" % [",".join(fields), request.item.id])
                row = cur.fetchone()
                if row is None:
                    return self.write_response(404, "not found", response)
                else:
                    for i, f in enumerate(fields):
                        setattr(response.item, f, row[i])
                    return self.write_response(200, "ok", response)
            elif request.method == "POST":
                values = [getattr(request.item, f) for f in fields]
                cursor.execute(self.node.create_upsert_query(
                        table="ProjectItem",
                        pkey=["id"],
                        cols=fields,
                    ), dict(zip(fields, values)))
                self.con.commit()
                return self.write_response(200, "ok", response)
            else:
                return self.write_response(403, "Forbidden method: %s" % request.method, response)
        except Exception as e:
            self.get_logger().error(str(e))
            return self.write_response(500, "internal error", response)
        finally:
            cursor.close()

def main(args=None):
    rclpy.init(args=args)
    db_server = DBServer()
    rclpy.spin(db_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
