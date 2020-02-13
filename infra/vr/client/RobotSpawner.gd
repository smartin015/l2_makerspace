extends Spatial

var sdf = load("res://addons/sdf/main.gd").new()

var control_zone = """
<?xml version="1.0"?>
<sdf version="1.4">
  <model name="ExampleControlZone">
  <link name="L2ControlZone">
    <pose>0.1 0 0 0.1 0 0</pose>
    <inertial>
      <mass>not_read</mass>
    </inertial>
    <collision>
      <geometry>
        <box><size>0.05 0.05 0.05</size></box>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box><size>0.05 0.05 0.05</size></box>
      </geometry>
      <material>
        <script>Gazebo/Purple</script>
      </material>
      <transparency>0.1</transparency>
    </visual>
  </link>
  </model>
</sdf>
"""

# Called when the node enters the scene tree for the first time.
func _ready():
  # Load example control zone; commented out because SDFs don't bundle :(
  # var f = File.new()
  # f.open("res://control_zone.sdf", File.READ)
  # var t = f.get_as_text()
  # f.close()
  var models = sdf.ParseAttrs(control_zone)
  for m in models.values():
    self.add_child(m)
    print("Added SDF model with ", m.get_children()[0].get_children()[0])
