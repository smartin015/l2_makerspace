extends SceneTree

var streamer = load("res://addons/poly/MeshStreamer.gd").new()

func _on_mesh_loaded(mesh):
  print("Got mesh %s", mesh)
  # get_tree().get_node('/root/World').add_child(mesh.instance())
  quit()
  
func _init():
  get_root().add_child(streamer)
  var err = streamer.connect("mesh_loaded", self, "_on_mesh_loaded")
  if err != OK:
    print("Err connecting to mesh_loaded: ", err)
  streamer.asset_search("piano")
  


