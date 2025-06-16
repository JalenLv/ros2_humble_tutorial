#!/usr/bin/env bash

if [ "$#" -eq 0 ]; then
    colcon build --mixin compile-commands #ninja
else
    colcon build --packages-select "$@" --mixin compile-commands #ninja
fi

