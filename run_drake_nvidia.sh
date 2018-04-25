#!/bin/bash


xhost +local:root; nvidia-docker run -i --rm -e DISPLAY \
-e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
--privileged -v "$(pwd)"":"/notebooks -t drake bash; xhost -local:root
