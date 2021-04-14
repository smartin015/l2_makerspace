Ideas for LCD display:

*   Teensy3.5 sending/receiving data back and forth to ESP32 wifi display

Substantial noise from J4 (smaller stepper driver) causing skips in encoder pulse responses

*   Root cause: noise from stepper lines causing interference in encoder wires. Routed stepper lines farther away from encoders and shielded them with metal fiber sleeving to reduce noise - works fine now!


## 2021-04-13

Trying out webots simulation servers

[http://l2:1999/monitor](http://l2:1999/monitor) - monitors connected sim severs (this is the session server?)

[http://l2:2000/monitor](http://l2:2000/monitor) - monitors the simulation including some fancy grahping

[http://l2:1999/session](http://l2:1999/session) - the address of the least loaded sim server

Could edit the config/setup at `$WEBOTS_HOME/resources/web/server/config/session/default.json`


## 2021-04-08

Made lots of fixes in the motion control firmware - a lot of silly mistakes made in variable naming. Switching to floating point also helps a lot (the Teensy has an FPU, so this should be reasonably performant).

Stub robot environment was crashing for some reason


## 2021-04-02

Added websocket reconnect loop and some color indication - would be great to see a real time chart of the position though, for forensics. [https://apexcharts.com/javascript-chart-demos/line-charts/realtime/](https://apexcharts.com/javascript-chart-demos/line-charts/realtime/) would do that.

Maybe a rolling buffer of step details too? For playback / debugging. "Breakpoint" or internal state dump at certain positions?


## 2021-03-30

Back into it! Starting to make it easier to launch the software environment. Going with a script in /ar3 that triggers multiple docker compose files. Not done yet, partial progress in tmux.

Current goal is to have an environment where I can save a file, run a single command, and get a new, running build from scratch in under 5 seconds. Need to:



1. Create script to start the web UI, the firmware sim, and the stub robot environment
2. Add websocket reconnect loop to web UI (refreshes whole page when connection re-established)


## 2021-03-15

Figured out how to get the firmware native execution to terminate - just required actually listening to the SIGINT signal when Ctrl+C pressed, then explicitly exiting the program. Weird that this wasn't done automatically.

Kalman filter impl in arduino code: https://bitbucket.org/adamb3_14/servoproject/src/master/ArduinoSketch/KalmanFilter.cpp


## 2021-03-14

After a lot of debugging.... get_steps returned a bool >.&lt;

Can now do the full-sim test setup from 2021-03-12 :D

But encoder limits aren't properly calibrated, and steps aren't converted to angles properly. That's the next step.


## 2021-03-12

Got webots working again via container - it was an issue with Mesa GL version compatibility where the container kept getting OpenGL 3.1 because that was the best it thought it could provide. Using `MESA_GL_VERSION_OVERRIDE: 3.3`  appeased webots which had a hard cutoff below that version.

Recovering state on the firmware simulation... it was crashing because the ZMQ REP/RES paradigm wasn't upheld. Changed firmware code to only send if it received, which fixed that.

Test code to drive the firmware is located at `.../ar3/firmware/test/native_client.py`. Changing this to accept keypresses to move each joint, since hand-typing a message is cumbersome.

...but actually, it's impossible to detect "keyup" in a terminal. So building a web app instead, hooray.

Now .../native_client.py opens a web page at port 8000 and communicates via websocket on port 8001. Set "--loopback" to test just the web functionality without having a connected client.

The setup that I'll need to verify:



1. In "sim" folder: `docker-compose run main`, then `ros2 run l2_ar3 node`
    1. Starts simulation environment (no webots). Make sure the container has been built recently (or else use the symlink hack in README.md)
2. In "firmware" folder: `docker-compose run native`
    2. Starts firmware controller
3. In "firmware/test" folder: python3 native_client.py
    3. Starts web control service


## 2021-03-11

Trying to run sim environment via docker (requiring GPU) but with also docker-compose, needed to reinstall docker-compose to get version 1.28.5 for the [new gpu config spec](https://docs.docker.com/compose/gpu-support/).

Looks like two container images in .../ar3/sim: 



*   docker build -t l2moveit -f ./Dockerfile.moveit .
*   docker-compose build


## 2021-03-04

Okay, back to custom controller time. Let's put as little smarts as possible in the teensy, but enough to properly fit position and velocity.

Continuous input and output: [

  Xmask,Y...,...,Cmask

  Xtarget...

  X...

  X'...

]

4*6 values, assume int16 -> 48 bytes/packet, at 115200 baud -> 2400hz, bidirectional -> 1200hz, 0.8ms period theoretical max

Mask passed to teensy includes: stepper enable, stepper compliance(?), limit switch ignore

Mask returned from teensy includes: limit switch made, target reached

Use an LQR controller or PID controller to fit position & velocity from the raspi to the controller.

Then we build a  


## 2021-03-03

Installed ubuntu 20.04 - needed to connect to ethernet to enable wifi, see [https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line](https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line) 

Filesystem was not resized properly so ran through instructions at [https://ubuntu-mate.community/t/resizing-disc-on-raspberry-pi/2833/10](https://ubuntu-mate.community/t/resizing-disc-on-raspberry-pi/2833/10) with "resize2fs". Note that it's easier to do this without having booted the pi (i.e. on another machine)


#### MachineKit experimentation



1. Flash RPI with debian image
2. Install machinekit
    1. [https://www.machinekit.io/docs/getting-started/APT-packages-stretch/](https://www.machinekit.io/docs/getting-started/APT-packages-stretch/) to get packages
    2. Attempt install RT kernel with [http://repos.rcn-ee.com/debian/pool/main/l/linux-upstream/linux-firmware-image-4.4.68-ti-rt-r109_1buster_armhf.deb](http://repos.rcn-ee.com/debian/pool/main/l/linux-upstream/linux-firmware-image-4.4.68-ti-rt-r109_1buster_armhf.deb)
        1. IDK if this actually works... just used dpkg -i
    3. [https://www.machinekit.io/docs/getting-started/install-runtime-packages/](https://www.machinekit.io/docs/getting-started/install-runtime-packages/)
        2. sudo apt-get install machinekit-rt-preempt
    4. Bah doesn't work. Let's try from source
        3. sudo apt-get install build-essential fakeroot devscripts equivs
        4. Follow RIP instructions at [https://github.com/machinekit/machinekit-hal](https://github.com/machinekit/machinekit-hal)
        5. When running armv7l (raspbian), got an error: `mk-build-deps -irs sudo` --> failed installing machinekit-hal-build-deps @ /usr/bin/mk-build-deps line 416 

######################################################################

#                       	Machinekit HAL                       	#

#   Universal framework for machine control based on HAL principle   #

######################################################################

#                                                                	#

#   Machinekit is a universal software system for computer control   #

#   of wide spectrum of machines. Machinekit is released under the   #

#   GPL.  Check out http://www.machinekit.io for more details.   	#

#                                                                	#

#                                                                	#

#   It seems that ./configure completed successfully.            	#

#   If things don't work check config.log for errors & warnings  	#

#                                                                	#

#   Next compile by typing                                       	#

#     	make                                                   	#

#     	sudo make setuid                                       	#

#                                                                	#

#   Before running the software, set the environment:            	#

#     	. (top dir)/scripts/rip-environment                    	#

#                                                                	#

#   To run the software type                                     	#

#     	halrun                                                 	#

#                                                                	#

######################################################################



1. Instructions thereafter from [here](http://www.machinekit.io/docs/developing/machinekit-developing/) - ../scripts/check-system-configuration.sh
2. 
3. Run examples (?)
    5. Understand how outputs work, especially whether we need to build an RPI hat for stepper control or if we should instead go over serial


#### Machinekit is a mess - let's try LinuxCNC

sudo apt install -y devscripts

dh-python libudev-dev tcl8.6-dev tk8.6-dev asciidoc dblatex

libreadline-gplv2-dev docbook-xsl dvipng ghostscript graphviz

groff imagemagick inkscape python-lxml source-highlight

texlive-extra-utils texlive-font-utils texlive-fonts-recommended

texlive-lang-cyrillic texlive-lang-french texlive-lang-german

texlive-lang-polish texlive-lang-spanish texlive-latex-recommended

w3c-linkchecker xsltproc python-dev python-tk libxmu-dev

libglu1-mesa-dev libgtk2.0-dev intltool libboost-python-dev

libmodbus-dev libusb-1.0-0-dev desktop-file-utils yapps2

python-yapps bwidget libtk-img mesa-utils tclx8.4

tcl-tclreadline python-configobj python-gtkglext1

python-xlib gstreamer0.10-plugins-base netcat

python-pil python-glade2 python-pil.imagetk

python-gst-1.0 w3c-linkchecker libxmu-dev libtirpc-dev

gdebi gdebi-core

....giving up. 


## 2021-02-28


#### Recovering state

Coming back to this project after a pretty long break setting up the new house. Before the break, I was setting up simulated integration with the robotic arm, the display simulation, and hardware / firmware controller.

To start webots sim:


    docker run --gpus=all --name l2ar3 -it --rm -e DISPLAY -v $(pwd)/webots:/usr/local/webots/resources/projects/ -v $(pwd)/config:/root/.config/Cyberbotics/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd):/volume l2ar3sim /bin/bash


    webots

You can test webots in isolation using the simple_controller.py loaded in that environment

New terminal - exec into the container and run the controller node

	docker exec -it l2ar3 /bin/bash

	ros2 run l2_ar3 node

The sim controller communicates firmware details over zeromq - port 5556 to receive hardware step counts, 5557 to send limit switch state

New terminal - start the display sim


    docker run --gpus=all --name l2display -it --rm -e DISPLAY -v $(pwd):/volume -v


    /tmp/.X11-unix:/tmp/.X11-unix:rw l2display /bin/bash


    ./run_native.sh

The display looks on websockets port 8001 for data (config at .../display/data/secret.json). You can use the test websocket server to verify behavior in isolation:

	python3 test_ws.py

Note: doesn't seem like I've yet set up websocket-driven data from the webots controller to the display :'(

Re-started a thread with [alex@machinekoder.com](mailto:alex@machinekoder.com) - he uses MachineKit with ROS to drive robotics arms

[https://machinekoder.com/machinekit-ros-open-source-robots/](https://machinekoder.com/machinekit-ros-open-source-robots/) has a good breakdown on forms of hardware control strategy. He recommends installing MachineKit for the HAL and to connect it up with a ros_control node. There's some restrictions on using an RT kernel.

[https://github.com/machinekoder/hal_ros_control](https://github.com/machinekoder/hal_ros_control) is the actual custom code which glues together ros_control and machinekit HAL. 
