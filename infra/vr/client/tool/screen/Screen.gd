extends Spatial

onready var mi = $MeshInstance
onready var img = Image.new()
onready var itex = ImageTexture.new()

remote func set_tf(tf):
  transform = tf

remote func set_ws(ws):
  self.ws = ws

func _ready():
  mi.material_override = SpatialMaterial.new()

remote func setup(channel):
  ROSBridge.ros_connect(channel, 
    "webpImage", 
    self, "_webp_data_received", 
    "webp_sub", 
    true) # Raw
  print("Connected to channel %s" % channel)

func _webp_data_received(data, _id):
  print("Got data %s" % len(data))
  # https://docs.godotengine.org/en/stable/classes/class_image.html#class-image-method-create-from-data
  if typeof(data) != TYPE_RAW_ARRAY:
    print("Got wrong type:", typeof(data))
    return
  if len(data) == 0:
    return
    
  # Remove channel byte
  var err = img.load_webp_from_buffer(data.subarray(1, -1))
  if err != OK:
    print(err)
  else:
    print(img.get_size())
    itex.create_from_image(img)
    mi.material_override.albedo_texture = itex
  
