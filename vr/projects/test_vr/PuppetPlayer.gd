extends Spatial

var velocity = Vector3()

puppet var puppet_transform
puppet var puppet_vel = Vector3()

# Called when the node enters the scene tree for the first time.
func _ready():
	# var player_id = get_network_master()
	# $NameLabel.text = gamestate.players[player_id]
	puppet_transform = self.transform # Just making sure we initilize it

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	# sync to last known position and velocity
	transform = puppet_transform
	velocity = puppet_vel
	
	transform = transform.translated(velocity * delta)
	
	# It may happen that many frames pass before the controlling player sends
	# their position again. If we don't update puppet_pos to position after moving,
	# we will keep jumping back until controlling player sends next position update.
	# Therefore, we update puppet_pos to minimize jitter problems
	puppet_transform = transform
	puppet_vel = puppet_vel * 0.95
