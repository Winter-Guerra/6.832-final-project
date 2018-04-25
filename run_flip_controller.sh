#!/bin/bash

cd /drake

# Make sure quad stuff is compiled
bazel build examples/quadrotor:*

# Run the visualizer (backgrounded)
./bazel-bin/tools/drake_visualizer &

# Run the controller
./bazel-bin/examples/quadrotor/run_quadrotor_lqr

cd /notebooks