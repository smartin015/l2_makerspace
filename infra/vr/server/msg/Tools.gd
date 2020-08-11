# ROS topics published by tools
extends Node

const tool_topics = {
  "PutFile": "l2_msgs/msg/L2File",
}

class ToolTopic:
  var topic = ""
  var type = ""
  
  func advertisement(id):
    return { 
      "op": "advertise",
      "topic": topic,
      "type": type,
      "id": "%s_%s" % [id, topic.to_lower()],
    }

func _ready():
  for t in tool_topics:
    var tt = ToolTopic.new()
    tt.topic = t
    tt.type = tool_topics[t]
    ROSBridge.topics.push_back(tt)

  
