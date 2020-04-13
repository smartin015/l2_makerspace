const NODE_MODEL = "model"
const NODE_LINK = "link"
const NODE_JOINT = "joint"

var openRE
var closeRE
var isRE
var defUseRE
var params
var defs

func init(parameters = {}):
  openRE = RegEx.new()
  openRE.compile("(\\w+)\\s+[\\{\\[]")
  closeRE = RegEx.new()
  closeRE.compile("[\\]\\}]")
  isRE = RegEx.new()
  isRE.compile("^(\\w+)\\s+IS\\s+(\\w+)$")
  defUseRE = RegEx.new()
  defUseRE.compile("^(DEF|USE)\\s+(\\w+)$")
  params = parameters
  defs = {}
  

# Basic node that contains the data for a corresponding element
class L2Node extends Spatial:
  var type = ""
  var data = {}
  func get_class():
    return "L2Node"
  func to_string():
    return "%s: %s %s" % [type, data, get_children()]
  func debug():
    return _debug(self)
  func _debug(p):
    if p == null or (typeof(p) == TYPE_OBJECT and p.get_class() != "L2Node"):
      return null
    var result = {"_children": []}
    for k in p.data.keys():
      if typeof(p.data[k]) == TYPE_OBJECT and p.data[k].get_class() == "L2Node":
        result[k] = _debug(p.data[k])
      else:
        result[k] = p.data[k]
    for c in p.get_children():
      result["_children"].append(_debug(c))
    return result
  func clone():
    var dup = self.duplicate()
    for k in data.keys():
      var v = data[k]
      if typeof(v) == TYPE_OBJECT and v.get_class() == "L2Node":
        dup.data[k] = v.clone()
      else:
        dup.data[k] = v
    dup.data = data
    for c in get_children():
      dup.add_child(c.clone())
    return dup

func parseBody(tag, sub) -> Array:
  var result = []
  # Returns keys & values
  var fmt = sub.strip_edges().rstrip("{}]")
  if fmt == "":
    return []
  for r in fmt.split('\n'):
    var kv = r.strip_edges().split(" ")
    if len(kv) >= 2 and kv[1] == "IS":
      if !params.has(kv[2]):
        print("WARNING: no param for PROTO line: " + r.strip_edges())
      else:
        var v = params[kv[2]]
        if typeof(v) == TYPE_OBJECT and v.get_class() == "L2Node":
          v = v.clone()
        result.append([kv[0], v])
    else:
      result.append(kv)
  return result

func postProcess(tag, parsed):
  var result = L2Node.new()
  result.data["_type"] = tag
  var nextDef = null
  var assignNext = null
  # Each element in parsed is either a key/value pair or 
  # the result of a previous call to postProcess.
  for p in parsed:
    if typeof(p) == TYPE_OBJECT and p.get_class() == "L2Node":
      if nextDef != null:
        defs[nextDef] = p
        nextDef = null
      # Unwrap children
      if p.data["_type"] == "children":
        for c in p.get_children():
          p.remove_child(c)
          result.add_child(c)
      elif assignNext != null:
        result.data[assignNext] = p
        assignNext = null
      else:
        result.add_child(p)
    elif p[0] == "children":
      result.add_child(p[1])
    elif len(p) == 1: # Value is next in parsed
      assignNext = p[0]
    elif len(p) == 2: # key/value
      if p[0] == "DEF":
        nextDef = p[1]
      elif p[0] == "USE":
        if !defs.has(p[1]):
          print("WARNING: no def for PROTO line: " + p.join(" "))
        else:
          result.add_child(defs[p[1]].clone())
      else:
        if typeof(p[1]) == TYPE_STRING and p[1].is_valid_float():
          result.data[p[0]] = float(p[1])
        else:
          result.data[p[0]] = p[1]
        
    else: # Array of values
      var vs = []
      var i = 1
      while i < len(p):
        if typeof(p[i]) == TYPE_STRING and p[i].is_valid_float():
          vs.append(float(p[i]))
        else:
          vs.append(p[i])
        i += 1
      result.data[p[0]] = vs
  return result

func parseStructure(s, idx, tag):
  var parsed = []
  while idx < len(s):
    # Setup indices
    var oSearch = openRE.search(s, idx)
    var cSearch = closeRE.search(s, idx)
    var oIdx = len(s)
    var cIdx = len(s)
    if cSearch != null:
      cIdx = cSearch.get_end()
    var sIdx = cIdx
    if oSearch != null and oSearch.get_start() < sIdx:
      oIdx = oSearch.get_end()
      sIdx = oSearch.get_start() 
    
    # We have some body to parse; append parsed body fields
    if idx != sIdx:
      for b in parseBody(tag, s.substr(idx, sIdx-idx)):
        parsed.append(b)
    
    if oIdx < cIdx: # Descend into inner structure
      var ps = parseStructure(s, oIdx+1, oSearch.get_string(1))
      parsed.append(ps[0])
      sIdx = ps[1]
    elif cIdx < oIdx: # Exit this structure
      idx = cIdx+1
      break
    
    # Continue parsing this structure
    idx = sIdx
    
  return [postProcess(tag, parsed), idx]

func parse(s):
  var st = parseStructure(s, 0, "Root")
  return st[0]
