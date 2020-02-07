# 3D vr simulated makerspace

## Requirements

- Uses [godot engine](http://godotengine.org)
- Oculus Quest compatible VR
- Multiplayer (shows position)
- Dynamically loadable 3D assets
- Google Poly integration

As the VR code matures, we can bridge the gap between multiple physical makerspaces and offer a fast way to switch between makerspaces.

### User journeys

A user should be able to use the VR makerspace to:

- Interact with people in the physical space, and joining over VR
- See what's happening in the physical space (displays/monitors and objects without static assets)
- Interact with physical and virtual smart devices on the ROS network
- Interact with the results of generative/optimization code

Example: Teleoperation: User wants to control an armature to solder a PCB

1. User enters VR space by opening the app on the Quest
   - Client connects to godot server running on GKE, which loads current dynamic mesh representing the armature and also starts forwarding depth camera information about the PCB
2. User navigates over to table with 2 armatures and PCB
   - Standard navigation (teleport or slide)
3. User zooms into the work area to get a closer look
   - Transparent bounding box is defined in SDF, published to ROS topic
   - Reaches out with controller in semitransparent bounding box
   - Presses "A" button
   - View switches to 1:3 scale, table surface adjusted to arm height
   - Soldering iron on/off button, temperature etc on arm panel
   - Robot arms extend from packed-away state and become compliant
   - Teleop is prevented if nobody is physically in the space (safety)
4. User starts to solder
   - Presses "A" while hand is over VR manipulator pendant (above gripper)
   - The real arm follows the manipulator pendant
   - Grips with the trigger. Tapping the side grip locks the grip position
   - Grip position cannot be unlocked until trigger is close to same value
   - Thumbstick advances solder reel
   - Clicking thumbstick changes function (e.g. soldering iron / tweezers)
5. User pauses soldering, adjusts headset
   - Presses "B" button before adjusting headset, then repeat #4
6. User accidentally jerks/drops controller
   - Jerk detection kicks in, cancels user control (equivalent of #5)
7. User exits the workspace
   - Presses "B" button when not controlling arms
   - Arms, solder temp, etc all remain the same
8. User reenters workspace, ends in a dangerous position, then the app crashes
   - e.g. soldering iron tip plunged into table
   - Can take off headset, open app, and press "emergency stop" button
   - power is cut to all configured tools in the space

Example: Maintenance: virtual user tidying the space


Example: Generative design: user wants to guide an optimization algorithm

1. User enters VR space
   - Client connects to godot server, which syncs dynamic ROS state.
1. User selects an active optimization project
1. User compares among top N results
1. User closely examines a single result
1. User indicates preference of several results, and anti-preference

Example: Generative design: user starts an optimization algorithm

1. User enters VR space
1. User goes to "generator" work station and enters it
1. User imports parts created in FreeCAD or elsewhere
1. User sets up constaints and parameters
1. User starts the generator

Example: Instruction: physical user demonstrating physical thing

1. User positions mobile depth camera
2. User registers mobile depth camera
3. User defines presentation area
4. User speaks about project
5. User mutes/bans misbehaving VR guest
6. VR moderator mutes/bans guest

## References

![concept sketch](https://github.com/smartin015/l2_makerspace/raw/master/docs/images/sketch.jpeg)

![3d render](https://github.com/smartin015/l2_makerspace/raw/master/docs/images/makerspace_3d.png)

![floor plan](https://github.com/smartin015/l2_makerspace/raw/master/docs/images/floorplan.png)

## Dimensions

The main area is 32'x14', with a 20'x6' rectangular extension on the bottom right of the floor plan.

## Milestones

Basic infra: completing the following milestones sets the groundwork for a VR makerspace environment.

- (DONE) Basic 3D navigation in 3D model of makerspace
- (DONE) Buildable oculus quest VR APK using godot v3.2
- (DONE) VR exploration of 3D model of makerspace
- (DONE) Multiple VR actors in scene (multiplayer) 
- (DONE) Offline and online dynamic model loading
- (DONE) Communication with local ros network (movable pendant)
- (DONE) Compressed UDP depth camera streaming


