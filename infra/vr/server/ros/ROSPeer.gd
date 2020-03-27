# A ROSPeer represents an instance of a ROS bridge connected to the godot server.
# It's possible for a user to be interacting with multiple ROS instances at a
# time, e.g. mixing a physical makerspace and a simulated one,
# or even fusing together two different physical makerspaces.
#
# There is currently no namespacing/isolation between peers, so care must
# be taken to not collide topic names.

extends Node

var id
var socket
var setup_state = {} # Map of id from _connection_msgs to whether it succeeded

var setuptmr
func begin_init(id, sck):
  self.id = id
  self.socket = sck
  
  # It may take some time for the newly-connected ROS bridge to be responsive.
  # We repeatedly try to set up in case the attempts initially fail. 
  # The initializer will stop the timer when it finally succeeds.
  setuptmr = Timer.new()
  setuptmr.wait_time = 2.0
  setuptmr.connect("timeout", self, "_init_connection") 
  add_child(setuptmr)
  setuptmr.start()

func _init_connection():
  if !socket:
    return

  var to_send = []
  for msg in ROSBridge.connection_msgs(id):
    if !setup_state.get(msg.id, false):
      to_send.push_back(JSON.print(msg).to_utf8())
      setup_state[msg.id] = false

  if len(to_send) == 0:
    # Done initializing, no more attempts needed
    setuptmr.stop()
    return

  # First, always set the logging level to "none" to get all messages.
  # This lets us match OK responses with advertisements/subscriptions
  to_send.push_front(JSON.print({
    "op": "set_level",
    "id": str(id) + "_setlevel",
    "level": "none", 
  }).to_utf8())

  for pkt in to_send:
    socket.put_packet(pkt)
  print("ROS(%s) sent %d setup msgs" % [id, len(to_send)])

func handle_status(id, level, msg):
  if level == 'none' and msg == 'OK' and setup_state.has(id):
    setup_state[id] = true

func should_throttle(sender):
  # TODO per-user throttling publishes to ROS bridge
  return false
