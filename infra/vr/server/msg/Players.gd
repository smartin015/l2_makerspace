# ROS topic of current connected players
# 
# Test with:
#  ros2 topic echo /l2/vr/Players
extends Node

var polltmr
const TOPIC_TYPE = "l2_msgs/VRPlayers"
onready var players = find_node('/root/World/Players')

func _ready():
  ROSBridge.topics.push_back(self)
  polltmr = Timer.new()
  polltmr.wait_time = 30.0
  polltmr.connect("timeout", self, "_poll") 
  add_child(polltmr)
  polltmr.start()

func advertisement(id):
  return { 
    "op": "advertise",
    "topic": "Players",
    "type": TOPIC_TYPE,
    "id": "%s_players" % id,
  }

func _poll():
  var names = []
  for c in players.get_children():
    names.append(c.name)
  ROSBridge.publish("Players", TOPIC_TYPE, {
    "players": names,
   }, "players")
