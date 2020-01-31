extends Node

var server = TCP_Server.new()
const MESH_PORT = 4243
const MAX_CONNS = 3
const FILE_SIZE_LIMIT = 1024*1024

func _ready():
  if(server.listen(MESH_PORT,"0.0.0.0") != OK):
    print("An error occurred listening on port", MESH_PORT)
  else:
    print("Listening for depth imagery")
  
var conns = []
func _process(_delta):
  if server.is_listening() && server.get_incoming_connections() > 0 && len(conns) < MAX_CONNS:
    conns.push_back(server.take_connection())
  
  for conn in conns:
    # TODO actually handle connection data; get object name, convert to mesh,
    # pass the godot style mesh data to the clients.
    # Limit inbound file size by FILE_SIZE_LIMIT
    var filename = "TODO parse this"
    var length = FILE_SIZE_LIMIT + 1
    if length > FILE_SIZE_LIMIT:
      print("File size limit exceeded for file ", filename, " from peer TODO")
      conn.close() 
    print(conn.get_available_bytes())
  #  rset_unreliable("point_data", socket.get_var()) # PoolByteArray
