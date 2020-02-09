extends Spatial

func oq_can_static_grab(b, grab_area, controller, overlapping_bodies):
  return true

func set_pos(name: String, pos: Vector3):
  get_node(name).translation = pos
  if gamestate.is_initialized:
    rpc_unreliable_id(1, 'set_pos', name, pos)
