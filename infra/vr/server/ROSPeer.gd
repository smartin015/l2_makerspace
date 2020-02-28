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

  print("ROS(%s) <- sending unacked connection messages" % id)
  var num_sent = 0
  for msg in ROSBridge.connection_msgs(id):
    if !setup_state.get(msg.id, false):
      socket.put_packet(JSON.print(msg).to_utf8())
      num_sent += 1
  
  # TODO: if num_sent == 0: # Done initializing, no more attempts needed
  # Requires bugfix where ros2-web-bridge returns set_level message w/ no id
  setuptmr.stop()

func should_throttle(sender):
  # TODO per-user throttling publishes to ROS bridge
  return false
