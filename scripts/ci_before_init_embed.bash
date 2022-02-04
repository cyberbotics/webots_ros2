#!/usr/bin/env bash

export WEBOTS_RELEASE_VERSION=2022a
export WEBOTS_OFFSCREEN=1
export CI=1
export DEBIAN_FRONTEND=noninteractive
export QTWEBENGINE_DISABLE_SANDBOX=1
export QT_DEBUG_PLUGINS=1
export LIBGL_ALWAYS_INDIRECT=0 # required for OpenGL apps like rviz
export DISPLAY=:1
