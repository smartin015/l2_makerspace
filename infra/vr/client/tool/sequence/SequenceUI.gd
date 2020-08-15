extends Control

onready var nodes = $MarginContainer/VBoxContainer/Spacer/Nodes
onready var items = $MarginContainer/VBoxContainer/Controls/MarginContainer/Items
onready var status = $MarginContainer/VBoxContainer/MarginContainer/HBoxContainer3/Status
onready var seqItemNode = load("res://tool/sequence/SequenceItemNode.tscn")
onready var seqItem = load("res://tool/sequence/SequenceItem.tscn")
onready var editModal = $L2ParamEdit
var editableNode = null

var running = false

func _log(s):
  print(s)
  if status != null:
    status.text = s

export var seq_items = [] setget _set_seq_items
func _set_seq_items(si):
  for c in items.get_children():
    c.queue_free()
  for i in si:
    var inst = seqItem.instance()
    inst.text = i
    inst.params = si[i]
    items.add_child(inst)
    var err = inst.connect("create_item", self, "_on_sequence_item_pressed")
    if err != OK:
      _log("Error connecting seqItem: %s" % err)
      inst.queue_free()
      continue
    
  print("Seq items initialized")
  seq_items = items

func clear():
  for c in nodes.get_children():
    if c is GraphNode:
      c.queue_free()
  nodes.clear_connections()

func _ready():
  # TODO set from ROS
  _set_seq_items({
    "pub": {"PUBSTR": "test_str"}, 
    "test2": {},
  })
  # $MarginContainer/VBoxContainer/Spacer/Nodes.get_zoom_hbox().visible = false

func _on_GridContainer_connection_request(from, from_slot, to, to_slot):
  # TODO prevent creation of cycles
  if running:
    _log("Cannot change connections while running")
    return
    
  rpc("connect_node", from, from_slot, to, to_slot)

remotesync func connect_node(from, from_slot, to, to_slot):
  nodes.connect_node(from, from_slot, to, to_slot)

remote func status_update(status_map):
  for uid in status_map:
    var n = nodes.get_node(uid)
    if n != null:
      n.set_status_label(status_map[uid])

func _on_sequence_item_pressed(t, p={}):
  print(t)
  print(p)
  # ID from msec ticks is sloppy and could cause problems
  # later on. But hey, it's test code :) you're welcome, future me.
  rpc("create_sequence_item", t, str(OS.get_ticks_msec()), p)

func _on_sequence_item_node_edit_pressed(n):
  editModal.fill(n.title, n.params)
  editModal.visible = true

func _on_L2ParamEdit_close_edit(title, prev, next):
  editableNode.rpc("set_params", next)
  editModal.visible = false

remotesync func create_sequence_item(n, uid, params = {}):
  var i = seqItemNode.instance()
  i.name = uid
  i.title = n  
  i.params = params
  i.connect("edit_pressed", self, "_on_sequence_item_node_edit_pressed")
  nodes.add_child(i)
  _log("Added node %s" % n)
  return i

func _on_Left_pressed():
  print("TODO left")

func _on_Right_pressed():
  print("TODO right")

func _on_Run_pressed():
  var seqMap = {}
  for c in nodes.get_connection_list():
    if c.from_port != 0 or c.to_port != 0:
      _log("Error: weird port connection %s" % c)
      return
    if seqMap.get(c.from) != null:
      _log("Error: multiple edges out from %s" % c.from)
      return
    seqMap[c.from] = c.to

  var origins = {}
  var params = {}
  for c in nodes.get_children():
    if c is GraphNode:
      origins[c.name] = true
      params[c.name] = c.params
  for k in seqMap:
    origins[seqMap[k]] = false
  var seq = []
  for k in origins:
    if origins[k]:
      seq.append(k)
  if len(seq) != 1:
    _log("Error: need 1 start node, got: %s" % [seq])
    return
  
  print(seq[0])
  var cmds = [nodes.get_node(seq[0]).get_data()]
  while seqMap.get(seq[-1]):
    seq.append(seqMap[seq[-1]])
    cmds.append(nodes.get_node(seq[-1]).get_data())
  
  running = true
  rpc_id(1, "run_sequence", cmds)
  _log("Sent run request: %s" % cmds)

  
func _on_Stop_pressed():
  running = false
  rpc_id(1, "run_sequence", [])
  _log("Sent stop request")

func _on_Clear_pressed():
  clear()

func _on_Save_pressed():
  rpc_id(1, "save")
  
func _on_Load_pressed():
  print("TODO load")

remote func on_save(status, name):
  # Called by server when save action ends
  if status == OK:
    gamestate.player.show_toast("Saved to %s" % name)
  else:
    gamestate.player.show_toast("Error saving: %s" % status)

func _on_Nodes_node_selected(node):
  if editableNode != null:
    editableNode.on_unselected()
  node.on_selected()
  editableNode = node

func _on_Nodes_node_unselected(node):
  if node == editableNode:
    node.on_unselected()
    editableNode = null
