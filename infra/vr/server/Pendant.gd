extends Spatial

remote func set_pos(name: String, pos: Vector3):
  var caller_id = get_tree().get_rpc_sender_id()
  get_node(name).translation = pos
  for p_id in gamestate.players:
    if p_id != caller_id:
      rpc_unreliable_id(caller_id, "set_pos", name, pos)
  ROSBridge.publish_pendant_pos(name, pos)
