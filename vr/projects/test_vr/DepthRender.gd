extends ImmediateGeometry

var pts = PoolVector3Array()
var socket = PacketPeerUDP.new()
	
func _ready():
	var point_size = 5
	var m = SpatialMaterial.new()
	m.flags_use_point_size = true
	m.params_point_size = point_size
	self.set_material_override(m)
	
	if(socket.listen(4242,"127.0.0.1") != OK):
		print("An error occurred listening on port 4242")
	else:
		print("Listening on port 4242 on localhost")
	
func _process(_delta):
	if(socket.is_listening() && socket.get_available_packet_count() > 0):
		pts = socket.get_var()
		var err = socket.get_packet_error()
		if err:
			print(err)
		else:
			renderPointCloud()
	
func renderPointCloud():
	self.clear()
	self.begin(Mesh.PRIMITIVE_POINTS, null)
	for p in pts: #list of Vector3s
		self.add_vertex(p)
	self.end()
	pass # Replace with function body.
