extends Node

puppet var locks = {} setget _czl
func _czl(n):
  locks = n
  print(n)

func get(name):
  return locks.get(name)

func acquire(name):
  rpc_id(1, "acquire_lock", name)

func release(name):
  rpc_id(1, "release_lock", name)
