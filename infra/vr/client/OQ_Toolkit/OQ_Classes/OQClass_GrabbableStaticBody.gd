# Attach this script to any rigid body you want to be grabbable
# by the Feature_RigidBodyGrab
extends Spatial

class_name OQClass_GrabbableStaticBody


var target_node = null;
var delta_orientation = Basis();
var delta_position = Vector3();
var is_grabbed := false

export var is_grabbable := true

var last_reported_collision_pos : Vector3 = Vector3(0,0,0);

func grab_init(node):
  target_node = node	
  var node_basis = node.get_global_transform().basis;
  is_grabbed = true

func grab_release(node):
  is_grabbed = false
  target_node = null

func _integrate_forces(state):
  if (!is_grabbed): return
  if (!target_node): return
  var target_basis =  target_node.get_global_transform().basis * delta_orientation;
  var target_position = target_node.get_global_transform().origin# + target_basis.xform(delta_position);
  transform.basis = target_basis
  transform.position = target_position
