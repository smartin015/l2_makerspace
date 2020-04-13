extends Node


onready var sdfParser = load("res://addons/parse/sdf.gd").new()
onready var protoParser = load("res://addons/parse/proto.gd").new()
onready var Actor = load("res://actor/Actor.tscn")

puppet func spawn(name, objtype, config, tf, peer_id):
  print("%s Actor %s @ %s from ROS(%s)" % [objtype, name, tf.origin, peer_id])
  var inst = Actor.instance()
  inst.name = name
  inst.transform = tf
  match objtype:
    "SDF":
      inst.add_child(sdfParser.ParseAttrs(config))
    "PROTO":
      inst.add_child(protoParser.ParseAttrs(config))
    _:
      print("ERROR: No parser for objtype " + objtype)
      return
  inst.setup_controls()
  add_child(inst)
  return inst

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("Actor %s removed" % name)

func _ready():
  spawn("testjoint", "PROTO", """
    Robot {
      translation 0 1.7 -1
      children [
        HingeJoint {
          jointParameters HingeJointParameters {
            anchor 0 1 0
          }
          device [
            RotationalMotor {
              name "wheel1"
            }
          ]
          endPoint Solid {
            translation 0 0 0
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 1 0.8 1
                  roughness 1
                  metalness 0
                }
                geometry Box {
                   size 0.1 0.1 0.1
                 }
              }
            ]
          }
        }
      ]
    }
  }""", Transform.IDENTITY, 0)

  spawn("testproto", "PROTO", """
  #VRML_SIM V8.5 utf8
  WorldInfo {
  }
  Viewpoint {
    orientation 1 0 0 -0.8
    position 0.25 0.708035 0.894691
  }
  Background {
    skyColor [0.2 0.2 0.2]
  }
  Solid {
    translation 0 1.5 -1
    children [
      Shape {
        appearance PBRAppearance {
          baseColor 0 1 0
          roughness 0.2
          metalness 0
        }
        geometry Box {
          size 0.23 0.1 0.1
        }
      }
    ]
  }
  Solid {
    translation -0.3 1.5 -1
    children [
      Shape {
        appearance PBRAppearance {
          baseColor 1 0 0
          roughness 0.2
          metalness 0.5
        }
        geometry Sphere {
          radius 0.1
        }
      }
    ]
  }
  Solid {
    translation 0.3 1.5 -1
    children [
      Shape {
        appearance PBRAppearance {
          baseColor 0 0 1
          roughness 1.0
          metalness 0.0
        }
        geometry Cylinder {
          radius 0.1
          height 0.1
        }
      }
    ]
  }""", Transform.IDENTITY, 0)
