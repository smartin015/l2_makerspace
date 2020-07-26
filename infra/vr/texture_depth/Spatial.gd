extends Spatial


# Declare member variables here. Examples:
# var a = 2
# var b = "text"
var img
var im
var it
var shad = load("res://shader.tres")
const step = 4
const side = 256

func test_sine(img):
  img.lock()
  for x in range(0, side):
    for z in range(0, side):
      img.set_pixel(x, z, Color(
        0.5 * sin(PI / side * (x + z)) + 0.5, 
        0, 0))
  img.unlock()

func test_rect(img, x, y, size, col = Color.red):
  img.lock()
  for i in range(x, x+size):
    for j in range(y, y+size):
      img.set_pixel(i, j, col)
  img.unlock()

func _ready():
  im = ImmediateGeometry.new()
  add_child(im)
  im.clear()
  im.begin(Mesh.PRIMITIVE_POINTS, null)
  for x in range(0, side, step):
    for z in range(0, side, step):
      var p = Vector3(x/float(side), 0, z/float(side))
      im.set_uv(Vector2(p.x, p.z))
      im.add_vertex(p - Vector3(0.5, 0, 0.5))
  im.end()
  im.material_override = shad
  
  
  # https://godotforums.org/discussion/21551/dynamically-generating-textures-at-runtime-from-c
  img = Image.new()
  img.create(side, side, false, Image.FORMAT_RG8)
  it = ImageTexture.new()
  # FLAG_VIDEO_SURFACE lets us call it.set_data(img) without
  # having to create a new ImageTexture for every update
  it.flags = ImageTexture.FLAG_VIDEO_SURFACE
  it.create_from_image(img,0)
  im.material_override.set_shader_param("depth", it)
  
  img.fill(Color(0,0,0))
  test_rect(img, side/2, side/2, 1, Color(0, 1, 0))
  test_rect(img, 0, 0, 1, Color(5.0/256.0, 0, 0)) 
  # Some values cause lookup to fail
  test_rect(img, 12, 12, 1, Color(8.0/256.0, 0, 0))
  it.set_data(img)
  
  # If the texture already has data,
  # you can upload pixels partially by providing the sub-rectangle you edited
  # VisualServer.texture_set_data_partial(_texture.get_rid(), _image, min_x, min_y, size_x, size_y, dst_x, dst_y, 0, 0)
  
  print("Init")
