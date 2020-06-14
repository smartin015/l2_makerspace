extends Area

signal hit_event

func ui_raycast_hit_event(position, click, release):
  emit_signal("hit_event", position, click, release)
