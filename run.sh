#!/bin/bash

# 1. Allow XQuartz connections
xhost +localhost

# 2. Get your Mac's IP (Alternative if host.docker.internal fails)
# export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
# xhost + $IP

# 3. Build and start the container
docker compose up -d

# 4. Enter the container
docker compose exec ros_dev bash