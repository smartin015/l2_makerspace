extends Node2D

var color: Color = Color.blue
var stroke = 2.0
var shapeType: int # gamestate.SHAPE
# Interpretation differs based on shape type
var points: PoolVector2Array

# Note: this should be kept in sync with the client implementation.
# _draw() is removed since the server is headless.

func start_shape(shape: int, origin: Vector2, col: Color):
  shapeType = shape
  color = col
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

func clear():
  points = PoolVector2Array()

func to_svg():
  match shapeType:
    gamestate.SHAPE.PENCIL, gamestate.SHAPE.LINE:
      var path = ""
      for i in range(len(points)):
        var ml = "L"
        if i == 0:
          ml = "M" # Move-to instead of line-to
        path += "%s %s %s" % [ml, points[i][0], points[i][1]]
      return "<path d=\"%s\" stroke=\"#%s\" fill=\"transparent\" stroke-width=\"%s\"/>" % [path, color.to_html(), stroke] 
    gamestate.SHAPE.CIRCLE:
      return "<circle cx=\"%s\" cy=\"%s\" r=\"%s\" stroke=\"#%s\" fill=\"transparent\" stroke-width=\"%s\"/>" % [points[0].x, points[0].y, points[1].length(), color.to_html(), stroke]
    gamestate.SHAPE.RECTANGLE:
      # Rectangle may be drawn with a negative second point (e.g. click, then drag up-left)
      # Normalize here so we don't pass negative vals to SVG
      var size = points[1].abs()
      var xy = points[0] + ((points[1] - size) / 2)
      return "<rect x=\"%s\" y=\"%s\" width=\"%s\" height=\"%s\" stroke=\"#%s\" fill=\"transparent\" stroke-width=\"%s\"/>" % [xy.x, xy.y, size.x, size.y, color.to_html(), stroke]
    _:
      print("Unimplemented shape %s conversion to SVG" % shapeType)
      return ""
