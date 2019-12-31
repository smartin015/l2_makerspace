# ROS MSG transformation

## Background

ROS has several [libraries](https://github.com/ros?utf8=%E2%9C%93&q=gen) that plug into `genmsg`, 
which is a tool for automatically generating libraries that implement the schema of a ros `.msg` file.

## Objective

Generate libraries for `ROS msg` sufficent to interact with messages in typescript and postgres. In particular:

1. Create a `gents` library that generates a typed library for use with a typescript service
2. Create a `genpostgres` library that generates postgres [composite types](https://www.postgresql.org/docs/10/rowtypes.html)

In both cases, any dependencies (e.g. std_msgs) must be correctly resolved.

## Resources

* https://github.com/ros?utf8=%E2%9C%93&q=gen&type=&language=
* https://github.com/RethinkRobotics-opensource/gennodejs
