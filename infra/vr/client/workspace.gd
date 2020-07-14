extends Node

const DEFAULT = "0"
var workspaces = []
var ws_fields = {}
var new_ws_cb = null

remote func set_visible(visible_ws):
  workspaces = visible_ws.keys()
  ws_fields = visible_ws
  
func request(obj, method):
  new_ws_cb = [obj, method]
  rpc_id(1, "request")
  
remote func on_new(ws):
  new_ws_cb[0].call(new_ws_cb[1], ws)
  new_ws_cb = null
  
func edit(ws, fields):
  rpc_id(1, "edit", ws, fields)
