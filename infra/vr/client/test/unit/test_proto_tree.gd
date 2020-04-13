extends "res://addons/gut/test.gd"

var proto = load("res://addons/parse/proto_tree.gd").new()

func test_basic():
  proto.init({})
  var got = JSON.print(proto.parse("""Solid {
    translation 0 0 1
    rotation 0 1 0 0
  }""").debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [],
        "_type": "Solid",
        "translation": [
          0,
          0,
          1
        ],
        "rotation": [
          0,
          1,
          0,
          0
        ]
      }
    ],
    "_type": "Root"
  })
  assert(got == want)
  
#func test_basic_string_array():
#  proto.init({})
#  var got = JSON.print(proto.parse("""WorldInfo {
#    info [
#      "Stuff"
#      "Things"
#    ]
#  }""").debug())
#  var want = JSON.print({})
#  assert(got == want)
#
#func test_def_use_on_assign():
#  proto.init({})
#  var got = JSON.print(proto.parse("""
#    geometry DEF BOX0 Box {
#      size 0.23 0.1 0.1
#    }""").debug())
#  var want = JSON.print({})
#  assert(got == want)
  
func test_basic_children():
  proto.init({})
  var got = JSON.print(proto.parse("""Solid {
  children [
    Transform {
      translation 0 0 -0.27
    }
    Transform {
      translation 0 0 0.5
    }
  ]
}""").debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [
          {
            "_children": [],
            "_type": "Transform",
            "translation": [
              0,
              0,
              -0.27
            ]
          },
          {
            "_children": [],
            "_type": "Transform",
            "translation": [
              0,
              0,
              0.5
            ]
          }
        ],
        "_type": "Solid"
      }
    ],
    "_type": "Root"
  })
  assert(got == want)

func test_hinge_joint():
  proto.init({})
  var got = JSON.print(proto.parse("""
    HingeJoint {
      jointParameters HingeJointParameters {
        anchor 0.06 0 0.05
      }
      device [
        RotationalMotor {
          name "wheel1"
        }
      ]
    }""").debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [
          {
            "_children": [
              {
                "_children": [],
                "_type": "RotationalMotor",
                "name": "\"wheel1\""
              }
            ],
            "_type": "device"
          }
        ],
        "_type": "HingeJoint",
        "jointParameters": {
          "_children": [],
          "_type": "HingeJointParameters",
          "anchor": [
            0.06,
            0,
            0.05
          ]
        }
      }
    ],
    "_type": "Root"
  })
  assert(got == want)

func test_nested_children():
  proto.init({})
  var got = JSON.print(proto.parse("""Solid {
  children [
    Transform {
      translation 0 0 -0.27
      children [
        Solid {
          appearance Appearance {
            material Material {}
          }
        }
      ]
    }
  ]
}""").debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [
          {
            "_children": [
              {
                "_children": [],
                "_type": "Solid",
                "appearance": {
                  "_children": [],
                  "_type": "Appearance",
                  "material": {
                    "_children": [],
                    "_type": "Material"
                  }
                }
              }
            ],
            "_type": "Transform",
            "translation": [
              0,
              0,
              -0.27
            ]
          }
        ],
        "_type": "Solid"
      }
    ],
    "_type": "Root"
  })
  assert(got == want)

func test_def_use():
  proto.init({})
  var got = JSON.print(proto.parse("""
    Transform {
      children [
        DEF LEG_SHAPE Shape {
          geometry Box { size 0.075 0.52 0.075 }
        }
      ]
    }
    Transform {
      children [ USE LEG_SHAPE ]
    }
  """).debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [
          {
            "_children": [],
            "_type": "Shape",
            "geometry": {
              "_children": [],
              "_type": "Box",
              "size": [
                0.075,
                0.52,
                0.075
              ]
            }
          }
        ],
        "_type": "Transform"
      },
      {
        "_children": [
          {
            "_children": [],
            "_type": "Shape",
            "geometry": {
              "_children": [],
              "_type": "Box",
              "size": [
                0.075,
                0.52,
                0.075
              ]
            }
          }
        ],
        "_type": "Transform"
      }
    ],
    "_type": "Root"
  })
  assert(got == want)

func test_non_child_object():
  proto.init({})
  var got = JSON.print(proto.parse("""Shape {
    geometry Cylinder {
      height 0.1
      radius 0.4
    }
  }""").debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [],
        "_type": "Shape",
        "geometry": {
          "_children": [],
          "_type": "Cylinder",
          "height": 0.1,
          "radius": 0.4
        }
      }
    ],
    "_type": "Root"
  })
  assert(got == want)

func test_pass_obj_param():
  proto.init({
    "seatGeometry": proto.parse("""
      Cylinder {
      height 0.1
      radius 0.4
    }""").get_child(0),
  })
  var got = JSON.print(proto.parse("""geometry IS seatGeometry""").debug())
  var want = JSON.print({
    "_children": [],
    "_type": "Root",
    "geometry": {
      "_children": [],
      "_type": "Cylinder",
      "height": 0.1,
      "radius": 0.4
    }
  })
  assert(got == want)

func test_web_example1():
  # https://www.cyberbotics.com/doc/reference/proto-example
  proto.init({
    "name": "test",
    "translation": [0, 0, 0], 
    "rotation": [0, 0, 1, 0],
    "seatExtensionSlot": null,
    "seatColor": [0, 0.5, 0],
    "seatGeometry": proto.parse("""
      Cylinder {
      height 0.1
      radius 0.4
    }""").get_child(0),
    "legColor": [0, 0.5, 0],
  })
  var got = JSON.print(proto.parse("""
    Solid {
      translation IS translation
      rotation IS rotation
      children [
        Transform {
          translation 0 0 -0.27
          children IS seatExtensionSlot
        }
        Transform {
          translation 0 -0.35 0
          children [
            Shape {
              appearance Appearance {
                material Material { diffuseColor IS seatColor }
              }
              geometry IS seatGeometry
            }
          ]
        }
        Transform {
          translation 0.25 -0.65 -0.23
          children [
            DEF LEG_SHAPE Shape {
              appearance Appearance {
                material Material { diffuseColor IS legColor }
              }
              geometry Box { size 0.075 0.52 0.075 }
            }
          ]
        }
        Transform {
          translation -0.25 -0.65 -0.23
          children [ USE LEG_SHAPE ]
        }
        Transform {
          translation 0.25 -0.65 0.2
          children [ USE LEG_SHAPE ]
        }
        Transform {
          translation -0.25 -0.65 0.2
          children [ USE LEG_SHAPE ]
        }
      ]
      name IS name
    }""").debug())
  var want = JSON.print({
    "_children": [
      {
        "_children": [
          {
            "_children": [],
            "_type": "Transform",
            "translation": [
              0,
              0,
              -0.27
            ]
          },
          {
            "_children": [
              {
                "_children": [],
                "_type": "Shape",
                "appearance": {
                  "_children": [],
                  "_type": "Appearance",
                  "material": {
                    "_children": [],
                    "_type": "Material",
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ]
                  }
                },
                "geometry": {
                  "_children": [],
                  "_type": "Cylinder",
                  "height": 0.1,
                  "radius": 0.4
                }
              }
            ],
            "_type": "Transform",
            "translation": [
              0,
              -0.35,
              0
            ]
          },
          {
            "_children": [
              {
                "_children": [],
                "_type": "Shape",
                "appearance": {
                  "_children": [],
                  "_type": "Appearance",
                  "material": {
                    "_children": [],
                    "_type": "Material",
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ]
                  }
                },
                "geometry": {
                  "_children": [],
                  "_type": "Box",
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ]
                }
              }
            ],
            "_type": "Transform",
            "translation": [
              0.25,
              -0.65,
              -0.23
            ]
          },
          {
            "_children": [
              {
                "_children": [],
                "_type": "Shape",
                "appearance": {
                  "_children": [],
                  "_type": "Appearance",
                  "material": {
                    "_children": [],
                    "_type": "Material",
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ]
                  }
                },
                "geometry": {
                  "_children": [],
                  "_type": "Box",
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ]
                }
              }
            ],
            "_type": "Transform",
            "translation": [
              -0.25,
              -0.65,
              -0.23
            ]
          },
          {
            "_children": [
              {
                "_children": [],
                "_type": "Shape",
                "appearance": {
                  "_children": [],
                  "_type": "Appearance",
                  "material": {
                    "_children": [],
                    "_type": "Material",
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ]
                  }
                },
                "geometry": {
                  "_children": [],
                  "_type": "Box",
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ]
                }
              }
            ],
            "_type": "Transform",
            "translation": [
              0.25,
              -0.65,
              0.2
            ]
          },
          {
            "_children": [
              {
                "_children": [],
                "_type": "Shape",
                "appearance": {
                  "_children": [],
                  "_type": "Appearance",
                  "material": {
                    "_children": [],
                    "_type": "Material",
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ]
                  }
                },
                "geometry": {
                  "_children": [],
                  "_type": "Box",
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ]
                }
              }
            ],
            "_type": "Transform",
            "translation": [
              -0.25,
              -0.65,
              0.2
            ]
          }
        ],
        "_type": "Solid",
        "translation": [
          0,
          0,
          0
        ],
        "rotation": [
          0,
          0,
          1,
          0
        ],
        "name": "test"
      }
    ],
    "_type": "Root"
  })
  assert(got == want)
