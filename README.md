<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]


# Aerial Manipulator

Code for the simulation of the aerial manipulator 
- [Installation](#installation)
- [Launch](#launch)

## Installation
This is the explanation of how to get the code running on a local machine. First download conda on the local machine.
```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
```
Then, create a new Python 3.7 conda environment (e.g., named "py3-aerial_simulation") and
activate it:

```bash
conda create -n py3-aerial_simulation python=3.7
conda activate py3-aerial_simulation
```

Then install:

```bash
chmod +x install_python.sh
./install_python.sh
```
Due to the virtual env is necessary to configure the catkin workspace as follows:
```bash
catkin_make -DPYTHON_EXECUTABLE=~/miniconda3/envs/py3-aerial_simulation/bin/python
source devel/setup.bash
```
## Launch
This section shows how to execute the simulatation
```bash
roslaunch aerial_manipulator system_dynamics.launch
```
