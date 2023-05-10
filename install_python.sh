#!/bin/bash
echo "Installing Python libraries..."
pip install numpy
pip install scipy==1.7.2
pip install pyyaml
pip install -U rospkg
pip install empy
pip install PyQt5
pip install PySide2
pip install mat4py
pip install pyOpenSSL
pip install autobahn
pip install twisted
pip install tornado
pip install bson
pip install pymongo
pip install Pillow
sudo apt-get install ros-noetic-rosbridge-server
