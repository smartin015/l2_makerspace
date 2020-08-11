# NOTE: Much of the logic of Sequence is handled in SequenceUI.gd
# to allow for 2D development on just the UI without a 3D VR environment.

extends Spatial

const objtype = "SEQUENCE"
puppet var ws = workspace.DEFAULT
onready var sui = $OQ_UI2DCanvas/Viewport/SequenceUI

remote func set_tf(tf):
  transform = tf

remote func set_ws(ws):
  self.ws = ws

remote func setup_request():
  rpc_id(get_tree().get_rpc_sender_id(), "setup", sui.pack_state())
