# gazebo_tasks
Gazebo simulation of 5 everyday tasks for robotics

By Paulo Abelha

E-mail: p.abelha@abdn.ac.uk

If using this repository in your own research, please e-mail me to cite my work (the paper to cite is yet to be published).

This repository is part of my PhD. It contains the code and files for simulating 5 everyday tasks in Gazebo.
This was only tested on Ubuntu 14.04, but should work in other versions.

I suggest you follow these installation steps for Gazebo in order to run the rest of my code that connects to Gazebo.

0) Dependencies for compiling the tasks:
  - 0.0 Update packages:
    - `sudo apt-get update`
  - 0.1 Install cmake:
    - `sudo apt-get install cmake`
  - 0.1 Install libprotobuf:
    - `sudo apt-get install libprotobuf-dev`
    - `sudo apt-get install libprotobuf-c-dev`

1) Set up Gazebo
  - 1.1) Follow: http://gazebosim.org/tutorials?tut=install_ubuntu
  - 1.2) Gazebo will create a hidden folder ~/.gazebo in your home
  - 1.3) Create a new directory called 'gazebo_models':
    - `mkdir ~/.gazebo/gazebo_models`
  - 1.4) Run Gazebo to check everything is okay:
    - `gazebo`
    (this should open gazebo an empty world)
  - 1.5) Close Gazebo
  - 1.6) Add the cafe_table model to the gazebo models folder
    - `cp -r ~/gazebo_tasks/cafe_table ~/.gazebo/gazebo_models/cafe_table`
  
2) Set up a task (TASK_NAME is the name of the task you want to set up (e.g. scooping_grains))
  - 2.1) Open your bashrc file
    - `gedit ~/.bashrc &`
  - 2.2) Add the following two lines as new lines at the end of bashrc file
    - `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
    - `export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/gazebo_models:~/.gazebo/gazebo_models/TASK_NAME`
    - 2.2.1) Please make sure you add ~/.gazebo/gazebo_models: to the beggining of the model path
  - 2.3) Save and close your file. Now source it
    - `source ~/.bashrc`
  - 2.4) For every task you will need to add its model and code path, spearating with ':'
    - `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/.gazebo/gazebo_models/TASK_NAME/plugins/build::~/.gazebo/gazebo_models/TASK_NAME2/plugins/build`
    - `export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/gazebo_models:~/.gazebo/gazebo_models/TASK_NAME:~/.gazebo/gazebo_models/TASK_NAME2`
  - 2.5) Copy the task folder from this repository for TASK_NAME to the gazebo_models folder
    - `cp -r ~/gazebo_tasks/TASK_NAME ~/.gazebo/gazebo_models/TASK_NAME`
  - 2.6) Remove the build directory inside the task folder
    - `rm -r ~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
  - 2.7) Create a new build directory inside the task folder
    - `mkdir ~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
  - 2.8) Go inside the build folder
    - `cd ~/.gazebo/gazebo_models/TASK_NAME/plugins/build`
  - 2.9) Build the task code and create make files
    - `cmake ..`
  - 2.10) Compile the task code
    - `make`

3) Run a simulation
  - 3.1) Go to the task folder
    `cd ~/.gazebo/gazebo_models/TASK_NAME`
  - 3.2) Run the simulation for the demo tool that goes with the repository
    `gazebo -u TASK_NAME.world --verbose`
    - (-u will make Gazebo start paused)
    - (--verbose is nice to show you any problems with finding paths, installation etc.)
  - 3.3) Check the file to see parameters and models loaded
    - `gedit ~/.gazebo/gazebo_models/TASK_NAME/TASK_NAME.world &`
  
 Good simulating! :) 
 
