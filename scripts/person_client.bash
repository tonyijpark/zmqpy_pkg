#!/bin/bash

cd `rospack find zmqpy_pkg`
cd src
echo `pwd`
python person_client.py