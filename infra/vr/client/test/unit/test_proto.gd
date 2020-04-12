extends "res://addons/gut/test.gd"

var proto = load("res://addons/sdf/proto.gd").new()

func debugObj(p):
  if p == null:
    return null
  var result = {"_children": []}
  for k in p.data.keys():
    if typeof(p.data[k]) == TYPE_OBJECT and p.data[k].get_class() == "ProtoNode":
      result[k] = debugObj(p.data[k])
    else:
      result[k] = p.data[k]
  for c in p.get_children():
    result["_children"].append(debugObj(c))
  return result

func test_basic():
  proto.init({})
  var got = JSON.print(debugObj(proto.parse("""Solid {
    translation 0 0 1
    rotation 0 1 0 0
  }""")))
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
  
func test_basic_children():
  proto.init({})
  var got = JSON.print(debugObj(proto.parse("""Solid {
  children [
    Transform {
      translation 0 0 -0.27
    }
    Transform {
      translation 0 0 0.5
    }
  ]
}""")))
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

func test_nested_children():
  proto.init({})
  var got = JSON.print(debugObj(proto.parse("""Solid {
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
}""")))
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
  var got = JSON.print(debugObj(proto.parse("""
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
  """)))
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
  var got = JSON.print(debugObj(proto.parse("""Shape {
    geometry Cylinder {
      height 0.1
      radius 0.4
    }
  }""")))
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
  var got = JSON.print(debugObj(proto.parse("""geometry IS seatGeometry""")))
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
  var got = JSON.print(debugObj(proto.parse("""
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
    }""")))
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
