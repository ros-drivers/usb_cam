#!/usr/bin/bash

# This file should be removed once upstream repo used in test is fixed
# to not require this script
# https://github.com/antmicro/renode-linux-runner-action

IMAGE="$1"

if [ -f "$IMAGE" ]; then
    if [ $(realpath "$IMAGE") == "$(realpath images.tar.xz)" ]; then
        echo File already exists
    else
        cp "$IMAGE" ./images.tar.xz
    fi
else
    wget -q --no-verbose "$IMAGE" -O ./images.tar.xz;
fi