#!/bin/bash

cd `rospack find zmqpy_pkg`
cd src
echo `pwd`
python3 face_server.py