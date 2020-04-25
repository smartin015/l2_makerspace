extends Node

var locks = {}

remote func acquire_lock(name):
  var held = locks.get(name)
  var caller_id = get_tree().get_rpc_sender_id()
  
  # If the caller is asking to set a lock it already has, give them
  # a courtesy response
  if held == caller_id:
    rset_id(caller_id, "locks", locks)
    
  # Don't allow another person to take the lock if it's already held
  # by an active player
  if gamestate.players.find_node(str(held)) != null:
    print("GDT(%s) -> ignore_lock %s" % [caller_id, name])
    return

  locks[name] = caller_id  
  print("GDT(%s) -> acquire_lock %s" % [caller_id, name])
  rset("locks", locks)
  
remote func release_lock(name):
  var held = locks.get(name)
  var caller_id = get_tree().get_rpc_sender_id()
  if held != caller_id:
    return
  locks[name] = null
  print("GDT(%s) -> release_lock %s" % [caller_id, name])
  rset("locks", locks)
