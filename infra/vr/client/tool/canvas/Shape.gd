extends Area2D

var color: Color = Color.blue
var stroke = 2.0
var pickable = false setget _set_pickable
onready var col = $CollisionShape2D
var shapeType: int # gamestate.SHAPE
# Interpretation differs based on shape type
var points: PoolVector2Array

func _set_pickable(p):
  pickable = p
  update()

func _draw():
  if pickable:
    var radius = get_node("CollisionShape2D").shape.radius
    # TODO Find centroid and set pickable location
    var p = points[0]
    draw_circle(p, radius, Color.yellowgreen)
    col.position = p    
    
  # https://docs.godotengine.org/en/stable/classes/class_canvasitem.html
  match shapeType:
    gamestate.SHAPE.PENCIL, gamestate.SHAPE.LINE:
      draw_multiline(points, color, stroke)
    gamestate.SHAPE.RECTANGLE:
      # 1st point position, 2nd point size
      draw_rect(Rect2(points[0], points[1]), color, false, stroke)
    gamestate.SHAPE.CIRCLE:
      # 1st point position, 2nd point edge point
      draw_arc(points[0], 
        points[1].length(), 
        0, PI*2, 24, color, stroke)
    
func start_shape(shape: int, origin: Vector2):
  print("Starting %s" % shape)
  shapeType = shape
  clear()
  match shapeType:
    gamestate.SHAPE.PENCIL:
      points = PoolVector2Array([origin])
    gamestate.SHAPE.LINE:
      points = PoolVector2Array([origin, origin])
    gamestate.SHAPE.RECTANGLE, gamestate.SHAPE.CIRCLE:
      points = PoolVector2Array([origin, Vector2(0,0)])
    _:
      print("Unimplemented shape type %s" % str(shapeType))
    
func handle_point(p: Vector2):
  match shapeType:
    gamestate.SHAPE.PENCIL:
      points.push_back(p)
    gamestate.SHAPE.LINE:
      points[1] = p
    gamestate.SHAPE.RECTANGLE, gamestate.SHAPE.CIRCLE:
      var w = p - points[0]
      points[1] = w
  update()

func clear():
  points = PoolVector2Array()


func _on_Area2D_input_event(viewport, event, shape_idx):
  if event is InputEventMouseButton:
    if event.is_pressed():
      print("Objecty click")
    else:
      print("Unclick")
