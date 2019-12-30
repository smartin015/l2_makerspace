# Makerspace Docker VDI

* Make sure nxserver is not running on the host (`sudo /usr/NX/bin/nxserver --shutdown`)
* Run `docker-compose up` to start a single VDI server
* Username and password are determined inside the yml file.
* Run `/usr/NX/bin/nxplayer`. It should detect the new host and allow you to connect. `--config` may prove handy as a way to automatically run the player.
