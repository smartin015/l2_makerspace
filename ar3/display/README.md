Based on https://github.com/lvgl/lv_platformio

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
