extends Spatial

var socket
const DEPTH_PORT = 4242

func _ready():
  socket = PacketPeerUDP.new()
  if(socket.listen(DEPTH_PORT,"0.0.0.0") != OK):
    print("An error occurred listening on port ", DEPTH_PORT)
  else:
    print("Listening for depth imagery on port ", DEPTH_PORT)
  
func _process(_delta):
  if socket.is_listening() && socket.get_available_packet_count() > 0:
    rset_unreliable("point_data", socket.get_var()) # PoolByteArray
