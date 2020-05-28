extends Spatial

func setup(topic: String):
  ROSBridge.ros_connect("0", 
    "webpImage", 
    self, "_webp_data_received", 
    "webp_sub", 
    true) # Raw

func _point_data_received(data, _id):
  if typeof(data) != TYPE_RAW_ARRAY:
    print("Got wrong type:", typeof(data))
    return
  if len(data) == 0:
    return
  
