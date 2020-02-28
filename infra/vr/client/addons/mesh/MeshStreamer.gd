extends Node

var poly_api_key
var outbound
const O_DOWN = "poly_download"
var down_file
const O_LIST = "poly_list"
const O_GET = "poly_get"
const POLY_GET_FMT = "https://poly.googleapis.com/v1/assets/%s?key=%s"
const POLY_LIST_FMT = "https://poly.googleapis.com/v1/assets?keywords=%s&key=%s&format=OBJ"
const API_KEY_PATH = "res://poly_api_key.secret"

var mat
var objparse = load("res://addons/mesh/ObjParse.gd").new()

onready var req = HTTPRequest.new()

var to_load = []

remote func spawn(asset_id):
  $MeshStreamer.stream(asset_id)

func _on_mesh_loaded(mi: MeshInstance):
  mi.transform.origin = Vector3(3, 1, 0)
  add_child(mi)

func _init_poly_api():
  var f = File.new()
  var err = f.open(API_KEY_PATH, File.READ)
  if err != OK:
    print("error opening poly api key file:", err)
    return
  poly_api_key = f.get_as_text().strip_edges()
  print("Poly API Key", poly_api_key)
  f.close()

func _init_httprequest():
  self.add_child(req)
  var err = req.connect("request_completed", self, "_on_request_completed")
  if err != OK:
    print("error registering request_completed: ", err)

func _ready():
  _init_httprequest()
  _init_poly_api()
  mat = SpatialMaterial.new()
  mat.albedo_color = Color(1, 1, 1)
  mat.flags_transparent = false
  mat.depth_enabled = false

func _load_next():
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
        _load_next()
  elif outbound == O_GET:
    var json = JSON.parse(body.get_string_from_utf8())
    if json.error != OK:
      print("couldn't parse json on line %d: %s\nbody\n%s" % [json.error_line, json.error_string, body.get_string_from_utf8()])
      return
    for fmt in json.result.formats:
      if fmt.formatType == "OBJ":
        print("%s %s" % [fmt.formatType, fmt.root])
        #for res in fmt.resources:
        #  to_load.push_back([res.relativePath, res.url])
        to_load.push_back([fmt.root.relativePath, fmt.root.url])
        _load_next()
  elif outbound == O_DOWN:
    print(O_DOWN, " ", response_code)
    var mesh = objparse.parse(body.get_string_from_utf8(), mat)
    var mi = MeshInstance.new()
    mi.mesh = mesh
    _on_mesh_loaded(mi)

func search_and_load(params):
  outbound = O_LIST
  var url = POLY_LIST_FMT % [params, poly_api_key]
  print(url)
  req.request(url) 

func stream(asset_id):
  outbound = O_GET
  var url = POLY_GET_FMT % [asset_id, poly_api_key]
  print(url)
  req.request(url) 
  
