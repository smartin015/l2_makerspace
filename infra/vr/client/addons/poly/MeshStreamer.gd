extends Node

var poly_api_key
var outbound
const O_DOWN = "poly_download"
var down_file
const O_LIST = "poly_list"
const POLY_LIST_FMT = "https://poly.googleapis.com/v1/assets?keywords=%s&key=%s&format=OBJ"

var objparse = load("res://addons/poly/ObjParse.gd").new()

signal mesh_loaded
onready var req = HTTPRequest.new()

var to_load = []

func load_next():
  var tl = to_load.pop_front()
  outbound = O_DOWN
  down_file = tl[0]
  req.request(tl[1])
  print("fecthing ", down_file)

func _on_request_completed(result, response_code, headers, body):
  if outbound == O_LIST:
    var json = JSON.parse(body.get_string_from_utf8())
    if json.error != OK:
      print("couldn't parse json on line %d: %s\nbody\n%s" % [json.error_line, json.error_string, body.get_string_from_utf8()])
      return
    for fmt in json.result.assets[0].formats:
      if fmt.formatType == "OBJ":
        print("%s %s" % [fmt.formatType, fmt.root])
        #for res in fmt.resources:
        #  to_load.push_back([res.relativePath, res.url])
        to_load.push_back([fmt.root.relativePath, fmt.root.url])
        load_next()
  elif outbound == O_DOWN:
    print(response_code)
    var mesh = objparse.parse(body.get_string_from_utf8())
    emit_signal('mesh_loaded', mesh)
    #var dat_out = File.new()
    #dat_out.open('user://' + down_file, File.WRITE)
    #dat_out.store_buffer(body)
    #dat_out.close()
    #if len(to_load) > 0:
    #  load_next()
    #else:
    #  
    #  var mesh=load('user://' + down_file)
    #  print(mesh)
    #  if mesh != null:
    #    emit_signal('mesh_loaded', mesh)

func _ready():
  self.add_child(req)
  var f = File.new()
  var err = f.open("res://poly_api_key.secret", File.READ)
  if err != OK:
    print("error opening poly api key file:", err)
    return
  poly_api_key = f.get_as_text().strip_edges()
  print("Poly API Key", poly_api_key)
  f.close()
  
  err = req.connect("request_completed", self, "_on_request_completed")
  if err != OK:
    print("error registering request_completed: ", err)
    
  asset_search('piano')
    
func asset_search(params):
  outbound = O_LIST
  var url = POLY_LIST_FMT % [params, poly_api_key]
  print(url)
  req.request(url) 

