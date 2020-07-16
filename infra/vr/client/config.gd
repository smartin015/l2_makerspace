extends Node

const CONFIG_PATH = "user://user_config.save"

static func default_user_config():
  var name_candidates = ["Robot", "Beaver", "Muskrat", "Chainsaw", "Pegleg", "Buttsmith99"]
  return {
    "alias": "Anon " + name_candidates[randi() % len(name_candidates)],
  }

static func load_user_config():
  var f = File.new()
  if not f.file_exists(CONFIG_PATH):
    print("No saved user config found, returning generated default")
    return default_user_config()

  f.open(CONFIG_PATH, File.READ)
  var data = JSON.parse(f.get_line())
  if data.error != OK:
    print("Failed to parse saved user config (%s), returning generated default" % data.error_string)
    return default_user_config()
  
  print("Loaded user config: %s" % data.result)
  return data.result
  
static func save_user_config(cfg):
  var f = File.new()
  f.open(CONFIG_PATH, File.WRITE)
  f.store_line(JSON.print(cfg))
  f.close()
  print("Saved user config")
