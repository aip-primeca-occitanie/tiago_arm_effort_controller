# Tiago_arm_controller

# SETUP
For this project, I have been forced to work on a virtual machine of the tiago development environment.
To reproduce my setup you will need :
- VMWare Workstation Player ([fr link](https://www.vmware.com/fr/products/workstation-player/workstation-player-evaluation.html)/[eng link](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html))
- a .iso file of the tiago development environment
- at least 16Gb of ram
- a good chair and some vanilla coffee ☕

First open VMWare Workstation Player (if you have trouble with access rights during the setup, run as administrator) and `Create a New Virtual Machine`:
```
> Install from: Installer disc image file (iso): 'Link the iso of the tiago develoment environment'
> Guest operating system: Linux > Version: Ubuntu 64-bits
> Virtual machine name: 'use your best naming skills ;)' > Location : C:\Users\...
> Maximum disk size (GB): depends on your needs, note that this is the MAXIMUM value, 
                          the virtual drives using dynamic allocation
> Customize Hardware:
  > Memory: 'select the maximum recommended memory, gazebo is very ressource demanding' 
  > Display: 3D graphics OFF (Gazebo doesn't support 3D acceleration in virtual machines)
```
Then save and launch the VM (Virtual Machine).
At the first start, select your language then select `Run Development TIAGO`.

> /!\ don't use `Install development environment` as you will have issues with drivers later on on the VM /!\

Use the installation drive on the desktop to install the TIAGO env.(environment) on your VM.
Restart the VM.
Now you should have an environment ready to simulate or deploy on a TIAGo !

> /!\ **do NOT do `sudo apt-get update`** as stated in the ros tutorials. Pal uses its own version of the librairies and packages and doing an update might break compatibility ! If packages are missing, look with `sudo apt search missing_package` and search for the pal repositories (either `pal-robotics/` or `pal-robotics-forks/`). /!\

# GRAVITY_COMPENSATION_CONTROLLER_TUTORIAL

The first test package you can install and run in simulation or deploy is the [gravity_compensation_controller_tutorial](https://github.com/pal-robotics/gravity_compensation_controller_tutorial) from Pal. This package serves as a learning tool to create our own controller !
 
First, create a workspace for your learning packages:
```
mkdir -p ~/learning_ws/src
```
Then clone the package from its source (here with SSH):
```
cd ~/learning_ws/src
git clone git@github.com:pal-robotics/gravity_compensation_controller_tutorial.git
```
...and build the package:
```
cd ~/learning_ws
catkin build gravity_compensation_controller_tutorial.git
```
For me, the build failed because of missing dependencies (namely rbdl and ddynamic_reconfigure).
To solve this type of problem, first look which is the correct dependency this error refers to:
```
sudo apt search rbdl
>>> drbdlinks/...
    pal-ferrum-rbdl/...
    pal-ferrum-rbdl-proneau/...
sudo apt search ddynamic_reconfigure
>>> pal-ferrum-ddynamic-reconfigure/...
    pal-ferrum-ddynamic-reconfigure-python/...
```
Here we are interested by `pal-ferrum-rbdl` and `pal-ferrum-ddynamic-reconfigure`.
Looking on git, we found the corresponding packages [pal-robotics-forks/rbdl](https://github.com/pal-robotics-forks/rbdl) and [pal-robotics/ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure), which we installed and built in our workspace:
```
cd ~/learning_ws/src
git clone git@github.com:pal-robotics-forks/rbdl.git
git clone git@github.com:pal-robotics/ddynamic_reconfigure.git
cd ..
catkin build rbdl ddynamic_reconfigure
```
Before retrying to build the `gravity_compensation_controller_tutorial`, be sure to delete the old failed build:
```
rm -rf ~/learning_ws/build/gravity_compensation_controller_tutorial
catkin build gravity_compensation_controller_tutorial.git
```
Now the package should have compiled and we can try to run it in simulation !
Before trying the simulation, we will add `source ~/learning_ws/devel/setup.bash` in the `.bashrc` since it will save us a lot of time:
```
echo "source ~/learning_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
We start by launching the tiago simulation in gazebo, after sourcing the workspace:
```
roslaunch tiago_gazebo tiago_gazebo.launch robot:=steel
```
> | Robot | Description |
> | --- | --- |
> | iron | no arm |
> | steel | arm with end_effector:='pal-gripper' |
> | titanium | arm with end_effector:='pal-hey5' |

and in a new terminal, we first stop the original arm_controller:
```
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'arm_controller'
strictness: 0" 
```
then we launch the gravity_compensation_controller:
```
roslaunch gravity_compensation_controller_tutorial gravity_compensation_controller_tutorial.launch simulation:=true robot:=tiago end_effector:=pal-gripper
```
For more informations, refer to the [original repo](https://github.com/pal-robotics/gravity_compensation_controller_tutorial) !
