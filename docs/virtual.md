# Virtual/physical interaction ideas

## Simulation

Probably the easiest of this list to set up - build a 3D reference model of the makerspace and add 
it to a multiplayer VR app for e.g. Oculus Quest. 

At first, this can be used to get a feel for what should live where in the space, and 
would help with development of tools for spectation / teleoperation and other VR interaction
in the space.

## Spectation

Perform live 3D scanning / depth imagery stitching of the makerspace - or specific parts of it - then overlay 
these scans onto the simulated environment above. VR users can watch live events, passively. 

With a little bit more effort, the positions and audio of the VR users can be projected into the space to 
allow for simple collaboration, question-answering etc.

## Teleoperation

Integrate with the moveit ROS framework (https://moveit.ros.org/) to allow remote VR manipulation of robots and
other devices in the makerspace.
