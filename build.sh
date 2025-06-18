#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON # -GNinja
else
    colcon build --packages-select "$@" --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON # -GNinja
fi

