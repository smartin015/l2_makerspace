extends Node

const CONFIG_PATH = "user://user_config.save"

static func default_user_config():
  var name_candidates = ["Robot", "Beaver", "Muskrat", "Chainsaw", "Pegleg", "Buttsmith99"]
  # https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
  var colors = ["#648FFF", "#785EF0", "#DC267F", "#FE6100", "#FFB000"]
  return {
    "alias": "Anon " + name_candidates[randi() % len(name_candidates)],
    "color": colors[randi() % len(colors)],
  }

static func load_user_config():
  var dc = default_user_config()
  var f = File.new()
  if not f.file_exists(CONFIG_PATH):
    print("No saved user config found, returning generated default")
    return dc

  f.open(CONFIG_PATH, File.READ)
  var data = JSON.parse(f.get_line())
  if data.error != OK:
    print("Failed to parse saved user config (%s), returning generated default" % data.error_string)
    return dc
  
  # Fill any unset defaults
  for k in dc:
    if not data.result.has(k):
      data.result[k] = dc[k]

  print("Loaded user config: %s" % data.result)
  return data.result
  
static func save_user_config(cfg):
  var f = File.new()
  f.open(CONFIG_PATH, File.WRITE)
  f.store_line(JSON.print(cfg))
  f.close()
  print("Saved user config")
