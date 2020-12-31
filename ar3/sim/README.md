
Dev environment:
```
 docker run --gpus=all --name l2ar3 -it --rm -e DISPLAY -v $(pwd)/webots:/usr/local/webots/resources/projects/ -v $(pwd)/config:/root/.config/Cyberbotics/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd):/volume l2ar3sim /bin/bash
```
