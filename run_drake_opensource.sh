#!/bin/bash

xhost +local:root; docker run -i --rm -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --privileged -v "$(pwd)"":"/notebooks -t drake_opensource bash; xhost -local:root
