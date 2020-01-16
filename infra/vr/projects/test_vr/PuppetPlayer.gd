extends Spatial

puppet var puppet_transform
puppet var transform_lhand
puppet var transform_rhand
puppet var transform_head
puppet var puppet_vel = Vector3()
puppet var velocity_lhand = Vector3()
puppet var velocity_rhand = Vector3()
puppet var velocity_head = Vector3()

onready var lhand = $LeftHand
onready var rhand = $RightHand
onready var head = $Head 

# Called when the node enters the scene tree for the first time.
func _ready():
	# var player_id = get_network_master()
	# $NameLabel.text = gamestate.players[player_id]
	puppet_transform = self.transform # Just making sure we initilize it
	transform_lhand = lhand.transform
	transform_rhand = rhand.transform
	transform_head = head.transform

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	# sync to last known position and velocity
	transform = puppet_transform
	lhand.transform = transform_lhand
	rhand.transform = transform_rhand
	head.transform = transform_head
	
	transform = transform.translated(puppet_vel * delta)
	lhand.transform = transform.translated(velocity_lhand * delta)
	rhand.transform = transform.translated(velocity_rhand * delta)
	head.transform = transform.translated(velocity_head * delta)
	
	
	# It may happen that many frames pass before the controlling player sends
	# their position again. If we don't update puppet_pos to position after moving,
	# we will keep jumping back until controlling player sends next position update.
	# Therefore, we update puppet_pos to minimize jitter problems
	puppet_transform = transform
	transform_lhand = lhand.transform
	transform_rhand = rhand.transform
	transform_head = head.transform
