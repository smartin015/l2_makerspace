extends Spatial

const objtype = "SCREEN"
var workspace = gamestate.DEFAULT_WORKSPACE
var chan = "1"

func _ready():
  pass

remote func setup_request():
  rpc_id(get_tree().get_rpc_sender_id(), "setup", chan)
