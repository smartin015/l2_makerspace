extends Node

func _ready():
  print("Control zone initialized")

func enter_zone(player):
  if locks.get(self.name) == null:
    locks.acquire(self.name)
  
func exit_zone(player):
  if locks.get(self.name) == get_tree().get_network_unique_id():
    locks.release(self.name)
