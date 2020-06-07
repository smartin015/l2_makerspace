extends Node2D

var color: Color = Color.blue
var stroke = 2.0
var shapeType: int # gamestate.SHAPE
# Interpretation differs based on shape type
var points: PoolVector2Array

# Note: this should be kept in sync with the client implementation.
# _draw() is removed since the server is headless.

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
  print("Handling %s" % shapeType)
  match shapeType:
    gamestate.SHAPE.PENCIL:
      points.push_back(p)
    gamestate.SHAPE.LINE:
      points[1] = p
    gamestate.SHAPE.RECTANGLE, gamestate.SHAPE.CIRCLE:
      var w = p - points[0]
      print("rect width %s" % w)
      points[1] = w

func clear():
  points = PoolVector2Array()
