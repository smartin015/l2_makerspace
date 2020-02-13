extends Spatial

func oq_can_static_grab(_b, _grab_area, _controller, _overlapping_bodies):
  return true

func set_pos_and_send(name: String, pos: Vector3):
  set_pos(name, pos)
  if gamestate.is_initialized:
    rpc_unreliable_id(1, 'set_pos', name, pos)

remote func set_pos(name: String, pos: Vector3):
  get_node(name).translation = pos
  
