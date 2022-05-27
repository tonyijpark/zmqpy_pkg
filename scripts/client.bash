#!/bin/bash

cd `rospack find zmqpy_pkg`
cd src
echo `pwd`
python image_sender.py