extends Spatial

remote func set_tf(tf):
  get_parent().transform = tf

remote func set_ws(ws):
  get_parent().ws = ws
