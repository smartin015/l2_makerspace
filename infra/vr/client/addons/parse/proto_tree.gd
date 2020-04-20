const NODE_MODEL = "model"
const NODE_LINK = "link"
const NODE_JOINT = "joint"

var openRE
var closeRE
var isRE
var defUseRE
var params
var defs

func re(s):
  var v = RegEx.new()
  v.compile(s)
  return v

func init(parameters = {}):
  openRE = re("[\\{\\[]")
  closeRE = re("[\\]\\}]")
  isRE = re("^(\\w+)\\s+IS\\s+(\\w+)$")
  defUseRE = re("^(DEF|USE)\\s+(\\w+)$")
  params = parameters
  defs = {}  

# Basic node that contains the data for a corresponding element
class L2Node extends Spatial:
  var data = {}
  func get_class():
    return "L2Node"

  func to_string():
    return "%s %s" % [data, get_children()]

  func debug():
    var result = {}
    for k in data.keys():
      result[k] = _debug(data[k])
      var v = data[k]
    if len(get_children()) > 0:
      result["_children"] = []
      for c in get_children():
        result["_children"].append(_debug(c))
    return result
  
  func _debugArray(arr):
    var r = []
    for a in arr:
      r.append(_debug(a))
    return r
      
  func _debug(v):
    match typeof(v):
      TYPE_OBJECT:
        if v.has_method("debug"):
          return v.debug()
      TYPE_ARRAY:
        var r = []
        for a in v:
          r.append(_debug(a))
        return r
    return v

  func clone():
    var dup = self.duplicate()
    var p = dup.find_parent("")
    if p:
      p.remove_child(dup)
    dup.data = {}
    for k in data.keys():
      var v = data[k]
      if typeof(v) == TYPE_OBJECT and v.get_class() == "L2Node":
        dup.data[k] = v.clone()
      else:
        dup.data[k] = v
    for c in get_children():
      dup.add_child(c.clone())
    return dup

func parseBody(sub, assign) -> Array:
  var result = []
  # Returns keys & values
  var fmt = sub.strip_edges().rstrip("[{}]")
  if fmt == "":
    return []
  for r in fmt.split('\n'): # TODO split by any whitespace
    r = r.strip_edges()
    if r == "":
      continue
    var kv = r.split(" ")
    if len(kv) >= 2 and kv[1] == "IS":
      if !params.has(kv[2]):
        print("WARNING: no param for PROTO line: " + r.strip_edges())
      else:
        var v = params[kv[2]]
        if v != null:
          if typeof(v) == TYPE_OBJECT and v.get_class() == "L2Node":
            v = v.clone()
          result.append([kv[0], v])
    elif len(kv) == 1:
      result.append(kv[0])
    else:
      result.append(kv)
    
  # Assign the last value
  if assign != null:
    result[-1] = [result[-1], assign]
  return result

func processValue(v):
  # Must always return either a singleton or an array of size 2
  if typeof(v) == TYPE_STRING:
    v = [v]
    
  if typeof(v) == TYPE_STRING_ARRAY:
    # Strip quotes and merge string
    # Unfortunately this loses some whitespace :-\
    if v[0][0] == '"' and v[-1][-1] == '"':
      v = [v.join(" ").trim_prefix('"').trim_suffix('"')]
    else:
      v = Array(v)
  
  if typeof(v) == TYPE_ARRAY:
    # Strip DEF
    var defname = null
    if typeof(v[0]) == TYPE_STRING_ARRAY:
      # Most complex form: "someParam DEF DEFNAME Object {}"
      # Makes v[0] = ["someParam", "DEF", "DEFNAME", "Object"]
      # We want to remove DEF, DEFNAME from any position and
      # return whatever's left
      var vnew = PoolStringArray()
      var i = 0
      while i < len(v[0]):
        if v[0][i] == "DEF":
          defname = v[0][i+1]
          i += 2
          continue
        vnew.append(v[0][i])
        i += 1
        
      # If there's only one value left, return it as a value (not array)
      if len(vnew) == 1:
        v[0] = vnew[0]
      else:
        v[0] = vnew
    
    # Replace USE
    if typeof(v[0]) == TYPE_STRING and v[0] == "USE":
      var n = v[1]
      v = [defs[n].clone()]
      v[0].data.erase("_def")
      v[0].data["_use"] = n
    
    for i in range(len(v)):
      if typeof(v[i]) == TYPE_STRING:
        if v[i].is_valid_float():
          v[i] = float(v[i])
        elif v[i][0] == '"' and v[i][-1] == '"':
          v[i] = v[i].trim_prefix('"').trim_suffix('"')
    if len(v) == 1: #unwrap from array
      return v[0]
    if len(v) == 2: #key/value, or [param+type]/node
      if typeof(v[1]) == TYPE_OBJECT and v[1].get_class() == "L2Node":
        # Assign key as type of node
        var node = v[1]
        var nodetype
        if typeof(v[0]) == TYPE_STRING_ARRAY:
          nodetype = v[0][-1]
        else:
          nodetype = v[0]
        node.data["_type"] = nodetype
        if defname != null:
          node.data["_def"] = defname
          defs[defname] = node
          defname = null
          
        if typeof(v[0]) == TYPE_STRING_ARRAY:
          return [v[0][0], node]
        return node
      else:
        return v
    else: #key/array
      var k = v[0]
      var val = []
      for i in range(1, len(v)):
        val.append(v[i])
      return [k, val]

  return v

func postProcessNode(parsed):
  var result = L2Node.new()
  var nextDef = null
  var assignNext = null
  # Each element in parsed is either a key/value pair or 
  # the result of a previous call to postProcess.
  for p in parsed:
    p = processValue(p)
    if typeof(p) == TYPE_OBJECT and p.get_class() == "L2Node":
      result.add_child(p)
    elif p[0] == "children":
      for c in p[1]:
        result.add_child(c)
    else:
      result.data[p[0]] = p[1]
  # print(JSON.print(result.debug()))
  return result

func getNextCloseIdx(s, idx):
  var m = closeRE.search(s, idx)
  if m != null:
    return m.get_end()
  return len(s)

func getNextOpenIdx(s, idx):
  var m = openRE.search(s, idx)
  if m != null:
    return m.get_end()
  return len(s)

func parseStructure(s, idx, is_array):
  # print(">>>: %s" % s.insert(idx, "|"))
  var parsed = []
  while idx < len(s):
    var closeIdx = getNextCloseIdx(s, idx)
    var openIdx = getNextOpenIdx(s, idx)
    
    # One of two cases can be true - either we have an 
    # inner nested structure coming up, or the end of this
    # structure.
    
    if openIdx < closeIdx: 
      # print(s[openIdx-1])
      # Parse the inner structure so we can properly assign
      # it when we parse the body.
      # There aren't any post-structure suffixes in VRML, which is
      # how we can do this in a loop.
      var ps = parseStructure(s, openIdx, s[openIdx-1] == "[")
      
      # Parse the body, being sure to assign ps
      # and add it to our parsed list
      for b in parseBody(s.substr(idx, openIdx-idx), ps[0]):
          parsed.append(b)
      
      # Update our index to past the parsed structure
      idx = ps[1]

    elif closeIdx <= openIdx: # No assignment
      # print(s[closeIdx-1])
      # Parse the body and add to our list
      if idx < closeIdx:
        for b in parseBody(s.substr(idx, closeIdx-idx), null):
          parsed.append(b)

      # Exit this structure
      idx = closeIdx+1
      break
    
  var result = []
  if is_array:
    for v in parsed:
      result.append(processValue(v))
  else:
    result = postProcessNode(parsed)
  return [result, idx]
  
func stripComments(s: String):
  var s2 = ""
  for line in s.split("\n"):
    var l = line.strip_edges(true, false) # left only
    if l != "" and l[0] == "#": 
      line = ""
    s2 += line + "\n"
  return s2
        
func parse(s: String):    
  var st = parseStructure(stripComments(s), 0, false)
  return st[0]
