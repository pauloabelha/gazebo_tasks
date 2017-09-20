# gazebo_tasks
Gazebo simulation of everyday tasks

This repository is part of my PhD. It contains the code anf files for simulating 5 everyday tasks in Gazebo.
This was only tested on Ubuntu 14.04, but should work in other versions.

I suggest you follow these installation steps for Gazbeo in order to run the rest of my code that connects to Gazebo.

1) Set up Gazebo
  - 1.1) Follow: http://gazebosim.org/tutorials?tut=install_ubuntu
  - 1.2) Gazebo will create a hidden folder ~/.gazebo in your home
  - 1.3) Create a new directory called 'gazebo_models':
    `mkdir ~/.gazebo/gazebo_models`
  - 1.4) Run Gazebo to check everything is okay:
    `gazebo`
    (this should open gazebo an empty world)
  
2) Set up a task (TASK_NAME is the name of the task you want to set up (e.g. scooping_grains))
  - 2.1) Open your bashrc file
    `gedit ~/.bashrc &`
  - 2.2) Add the following in a new lines at the end of bashrc file
    export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/.gazebo/gazebo_models/TASK_NAME/plugins/build
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/gazebo_models/TASK_NAME
  - 2.3) For every task you will need to add its model and code path, spearating with ':'
    `xport GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/.gazebo/gazebo_models/TASK_NAME/plugins/build::~/.gazebo/gazebo_models/TASK_NAME2/plugins/build`
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/gazebo_models/TASK_NAME:~/.gazebo/gazebo_models/TASK_NAME2
  - 2.4) Copy the task folder from this repository for TASK_NAME to the gazebo_models folder
    `cp -r ~/gazebo_tasks/TASK_NAME ~/.gazebo/gazebo_models/TASK_NAME`
  - 2.5) Remove the build directory inside the task folder
    `rm -r ~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
  - 2.6) Create a new build directory inside the task folder
    `mkdir ~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
  - 2.7) Go inside the build folder
    `cd ~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
  - 2.8) Build the task code and create make files
    `cmake ..`
  - 2.9) Compile the task code
    `make`

3) Run a simulation
  3.1) Go to the task folder
    cd ~/.gazebo/gazebo_models/TASK_NAME
  3.2) Run the simulation for the demo tool that goes with the repository
    gazebo -u TASK_NAME.world --verbose
    (-u will make Gazebo start paused)
    (-verbose is nice to show you any problems with finding paths, installation etc.)
  3.3) Check the ~/.gazebo/gazebo_models/TASK_NAME/TASK_NAME.world file to see parameters and models loaded
  
 Good simulating! :) 
 
