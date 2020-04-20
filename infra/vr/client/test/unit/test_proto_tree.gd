extends "res://addons/gut/test.gd"

var proto = load("res://addons/parse/proto_tree.gd").new()

func test_multistring_array():
  proto.init({})
  var got = JSON.print(proto.parse("""
    varthing [
      "Test string with multiple words"
      "Yet another test string"
    ]""").debug())
  var want = JSON.print({
      "varthing": [
        "Test string with multiple words",
        "Yet another test string"
      ]
    })
  assert(got == want)

func test_comment():
  proto.init({})
  var got = JSON.print(proto.parse("""
  # Comment here
  Solid {
    # Comment there
    translation 0 0 1
    # Comment comment
    rotation 0 1 0 0
  }
  # Everywhere""").debug())
  var want = JSON.print({
    "_children": [
      {
        "translation": [0,0,1],
        "rotation": [0,1,0,0],
        "_type": "Solid"
      }
    ]
  })
  assert(got == want)

func test_basic():
  proto.init({})
  var got = JSON.print(proto.parse("""Solid {
    translation 0 0 1
    rotation 0 1 0 0
  }""").debug())
  var want = JSON.print({
    "_children": [
      {
        "translation": [0,0,1],
        "rotation": [0,1,0,0],
        "_type": "Solid"
      }
    ]
  })
  assert(got == want)

func test_basic_string_array():
  proto.init({})
  var got = JSON.print(proto.parse("""WorldInfo {
    info [
      "Stuff"
      "Things"
    ]
  }""").debug())
  var want = JSON.print({
    "_children": [{
      "info": ["Stuff","Things"],
      "_type": "WorldInfo",
    }],
  })
  assert(got == want)

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
        "_type": "Solid",
        "_children": [
          {
            "translation": [
              0,
              0,
              -0.27
            ],
            "_type": "Transform"
          },
          {
            "translation": [
              0,
              0,
              0.5
            ],
            "_type": "Transform"
          }
        ]
      }
    ]
  })
  assert(got == want)

func test_middle_def():
  proto.init({})
  var got = JSON.print(proto.parse("""someValue DEF LEG_SHAPE Shape {}""").debug())
  var want = JSON.print({"someValue":{"_type":"Shape","_def":"LEG_SHAPE"}})
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
        "jointParameters": {
          "anchor": [
            0.06,
            0,
            0.05
          ],
          "_type": "HingeJointParameters"
        },
        "device": [
          {
            "name": "wheel1",
            "_type": "RotationalMotor"
          }
        ],
        "_type": "HingeJoint"
      }
    ]
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
        "_type": "Solid",
        "_children": [
          {
            "translation": [
              0,
              0,
              -0.27
            ],
            "_type": "Transform",
            "_children": [
              {
                "appearance": {
                  "material": {
                    "_type": "Material"
                  },
                  "_type": "Appearance"
                },
                "_type": "Solid"
              }
            ]
          }
        ]
      }
    ]
  })
  assert(got == want)

func test_def_use():
  proto.init({})
  var got = JSON.print(proto.parse("""
    Transform { 
      children [ 
        DEF LEG_SHAPE Shape {}
      ] 
    } 
    Transform { 
      children [ USE LEG_SHAPE ] 
    } 
  """).debug())
  var want = JSON.print({
    "_children": [
      {
        "_type": "Transform",
        "_children": [
          {
            "_type": "Shape",
            "_def": "LEG_SHAPE"
          }
        ]
      },
      {
        "_type": "Transform",
        "_children": [
          {
            "_type": "Shape",
            "_use": "LEG_SHAPE"
          }
        ]
      }
    ]
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
        "geometry": {
          "height": 0.1,
          "radius": 0.4,
          "_type": "Cylinder"
        },
        "_type": "Shape"
      }
    ]
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
    "_children": [
      {
        "height": 0.1,
        "radius": 0.4,
        "_type": "geometry"
      }
    ]
  })
  assert(got == want)

func test_single_motor():
  proto.init({})
  var got = JSON.print(proto.parse("""
    #VRML_SIM V8.5 utf8
    WorldInfo {
      info [           
        "Single motor example test"
        "Q2 2020"          
      ]                
    }           
    Viewpoint {     
      orientation 1 0 0 -0.8                    
      position 0.25 0.708035 0.894691
    }      
    Background {  
      skyColor [             
        0.4 0.4 0.4        
      ]      
    }                        
    PointLight {
      ambientIntensity 0.54
      intensity 0.5          
      location 0 1 0  
    }        
    DEF EXAMPLE Robot {
      name "example"
      controller "<extern>"
      translation 0 0 0
      children [
        HingeJoint {
          jointParameters HingeJointParameters {
            anchor 0 0 0
          }
          device [
            RotationalMotor {
              name "motor1"
            }
            PositionSensor {}
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
  """).debug())
  var want = JSON.print({
    "_children": [
      {
        "info": [
          "Single motor example test",
          "Q2 2020"
        ],
        "_type": "WorldInfo"
      },
      {
        "orientation": [
          1,
          0,
          0,
          -0.8
        ],
        "position": [
          0.25,
          0.708035,
          0.894691
        ],
        "_type": "Viewpoint"
      },
      {
        "skyColor": [
          [
            0.4,
            [
              0.4,
              0.4
            ]
          ]
        ],
        "_type": "Background"
      },
      {
        "ambientIntensity": 0.54,
        "intensity": 0.5,
        "location": [
          0,
          1,
          0
        ],
        "_type": "PointLight"
      },
      {
        "name": "example",
        "controller": "<extern>",
        "translation": [
          0,
          0,
          0
        ],
        "_type": "Robot",
        "_def": "EXAMPLE",
        "_children": [
          {
            "jointParameters": {
              "anchor": [
                0,
                0,
                0
              ],
              "_type": "HingeJointParameters"
            },
            "device": [
              {
                "name": "motor1",
                "_type": "RotationalMotor"
              },
              {
                "_type": "PositionSensor"
              }
            ],
            "endPoint": {
              "translation": [
                0,
                0,
                0
              ],
              "_type": "Solid",
              "_children": [
                {
                  "appearance": {
                    "baseColor": [
                      1,
                      0.8,
                      1
                    ],
                    "roughness": 1,
                    "metalness": 0,
                    "_type": "PBRAppearance"
                  },
                  "geometry": {
                    "size": [
                      0.1,
                      0.1,
                      0.1
                    ],
                    "_type": "Box"
                  },
                  "_type": "Shape"
                }
              ]
            },
            "_type": "HingeJoint"
          }
        ]
      }
    ]
  })
  assert(got == want)

func test_web_example1():
  # https://www.cyberbotics.com/doc/reference/proto-example
  proto.init({}) # to allow for the parse-within-init
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
        "name": "test",
        "_type": "Solid",
        "_children": [
          {
            "translation": [
              0,
              0,
              -0.27
            ],
            "_type": "Transform"
          },
          {
            "translation": [
              0,
              -0.35,
              0
            ],
            "_type": "Transform",
            "_children": [
              {
                "appearance": {
                  "material": {
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ],
                    "_type": "Material"
                  },
                  "_type": "Appearance"
                },
                "_type": "Shape",
                "_children": [
                  {
                    "height": 0.1,
                    "radius": 0.4,
                    "_type": "geometry"
                  }
                ]
              }
            ]
          },
          {
            "translation": [
              0.25,
              -0.65,
              -0.23
            ],
            "_type": "Transform",
            "_children": [
              {
                "appearance": {
                  "material": {
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ],
                    "_type": "Material"
                  },
                  "_type": "Appearance"
                },
                "geometry": {
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ],
                  "_type": "Box"
                },
                "_type": "Shape",
                "_def": "LEG_SHAPE"
              }
            ]
          },
          {
            "translation": [
              -0.25,
              -0.65,
              -0.23
            ],
            "_type": "Transform",
            "_children": [
              {
                "appearance": {
                  "material": {
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ],
                    "_type": "Material"
                  },
                  "_type": "Appearance"
                },
                "geometry": {
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ],
                  "_type": "Box"
                },
                "_type": "Shape",
                "_use": "LEG_SHAPE"
              }
            ]
          },
          {
            "translation": [
              0.25,
              -0.65,
              0.2
            ],
            "_type": "Transform",
            "_children": [
              {
                "appearance": {
                  "material": {
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ],
                    "_type": "Material"
                  },
                  "_type": "Appearance"
                },
                "geometry": {
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ],
                  "_type": "Box"
                },
                "_type": "Shape",
                "_use": "LEG_SHAPE"
              }
            ]
          },
          {
            "translation": [
              -0.25,
              -0.65,
              0.2
            ],
            "_type": "Transform",
            "_children": [
              {
                "appearance": {
                  "material": {
                    "diffuseColor": [
                      0,
                      0.5,
                      0
                    ],
                    "_type": "Material"
                  },
                  "_type": "Appearance"
                },
                "geometry": {
                  "size": [
                    0.075,
                    0.52,
                    0.075
                  ],
                  "_type": "Box"
                },
                "_type": "Shape",
                "_use": "LEG_SHAPE"
              }
            ]
          }
        ]
      }
    ]
  })
  assert(got == want)
