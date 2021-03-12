Based on https://github.com/lvgl/lv_platformio

Use `./run_docker.sh` to start the docker container before running any commands

Note: web config (including IP address and port for websockets server) configured
at ./data/secret.json. Wifi SSID & password aren't used in native-mode and can remain unset

```
# In a terminal, start the websocket server to generate positions
python3 test_ws.py

# In a separate terminal, start the build container
# Omit the /dev/ entry if you don't plan to program
./run_docker.sh /dev/ttyUSB0

# Then, when in docker, start the native (local UI) version of code
./run_native.sh

# Upload the code to the target board
python3 -m platformio run -e esp32dev -t upload

# Upload the config to the target board
python3 -m platformio run -e esp32dev -t uploadfs
```
