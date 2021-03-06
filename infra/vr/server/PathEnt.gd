enum Category {WORLD, GROUP, OWNER}
enum Access {READ, WRITE, EXECUTE}
# unix-style permission bitfield, rwx|rwx|rwx for owner|group|world
var perm = 0x000 
var perm_owner = ""
var perm_group = ""
var public = true
var name = ""
var fields = {} # Additional metadata, e.g. room skin for a workspace

func check(c, a):
  return (perm >> (3*int(c))) & (0x01 << int(a)) != 0

func set_name(c, n: String):
  if check(c, Access.WRITE):
    name = n

func get_state():
  return {"name": name, "fields": fields, "perm": perm}
