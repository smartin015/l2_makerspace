# Credit 
# https://poly.google.com/view/f8iZm3Vvfdm
# for the headset mesh

extends Spatial

var workspace
puppet var puppet_transform
puppet var puppet_motion

onready var left = $LeftHand
onready var right = $RightHand
onready var head = $Head 
onready var lskel = $LeftHand/OculusQuestHand_Left/ArmatureLeft/Skeleton
onready var rskel = $RightHand/OculusQuestHand_Right/ArmatureRight/Skeleton


const PM_HEAD = 0
const PM_LEFT = 3
const PM_RIGHT = 6

const PM_ORIGIN = 0
const PM_QUAT = 1
const PM_VEL = 2

func update_visibility():
  # Puppet is hidden - but not removed - to simplify
  # transmission of puppet_motion and prevent network errors.
  visible = (workspace == gamestate.player.workspace)

remote func set_workspace(ws):
  workspace = ws
  update_visibility()

func _ready():
  puppet_motion = [
    Vector3.ZERO, Quat.IDENTITY, Vector3.ZERO, # Head
    Vector3.ZERO, Quat.IDENTITY, Vector3.ZERO, # Left
    Vector3.ZERO, Quat.IDENTITY, Vector3.ZERO, # Right
  ]
  puppet_transform = transform
  puppet_hands(null, null)
  
# Copied from Feature_HandModel.gd
# we need to remap the bone ids from the hand model to the bone orientations we get from the vrapi and the inverse
# This is only for the actual bones and skips the tips (vrapi 19-23) as they do not need to be updated I think
const _vrapi2hand_bone_map = [0, 23,  1, 2, 3, 4,  6, 7, 8,  10, 11, 12,  14, 15, 16, 18, 19, 20, 21];

onready var lhand = $LeftHand/OculusQuestHand_Left
onready var rhand = $RightHand/OculusQuestHand_Right
onready var lcont = $LeftHand/OculusQuestTouchController_Left
onready var rcont = $RightHand/OculusQuestTouchController_Right
remote func set_hand_visibility(l: bool, r: bool):
  lhand.visible = l
  lcont.visible = not lhand.visible
  rhand.visible = r
  rcont.visible = not rhand.visible
  
remote func puppet_hands(lorient, rorient):
  if lorient != null:
    for i in range(0, len(_vrapi2hand_bone_map)):
      lskel.set_bone_pose(_vrapi2hand_bone_map[i], Transform(lorient[i]));
  if rorient != null:
    for i in range(0, len(_vrapi2hand_bone_map)):
      rskel.set_bone_pose(_vrapi2hand_bone_map[i], Transform(rorient[i]));
  set_hand_visibility(lorient != null, rorient != null)

func _process(delta):
  head.transform = Transform(puppet_motion[PM_HEAD+PM_QUAT], puppet_motion[PM_HEAD+PM_ORIGIN])
  head.transform = head.transform.translated(puppet_motion[PM_HEAD+PM_VEL] * delta)
  left.transform = Transform(puppet_motion[PM_LEFT+PM_QUAT], puppet_motion[PM_LEFT+PM_ORIGIN])
  left.transform = left.transform.translated(puppet_motion[PM_LEFT+PM_VEL] * delta)
  right.transform = Transform(puppet_motion[PM_RIGHT+PM_QUAT], puppet_motion[PM_RIGHT+PM_ORIGIN])
  right.transform = right.transform.translated(puppet_motion[PM_RIGHT+PM_VEL] * delta)
  
  # Many frames may pass before the controlling player sends more motion.
  # Updating position minimizes jitter problems
  puppet_motion[PM_HEAD+PM_ORIGIN] = head.transform.origin
  puppet_motion[PM_LEFT+PM_ORIGIN] = left.transform.origin
  puppet_motion[PM_RIGHT+PM_ORIGIN] = right.transform.origin
  if transform != puppet_transform:
    transform = puppet_transform
