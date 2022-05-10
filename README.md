# Tiago_arm_controller

# INSTALLATION
For this project, I have been forced to work on a virtual machine of the tiago development environment.
To reproduce my setup you will need :
- VMWare Workstation Player ([fr link](https://www.vmware.com/fr/products/workstation-player/workstation-player-evaluation.html)/[eng link](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html))
- a .iso file of the tiago development environment
- at least 16Gb of ram

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

> /!\ do NOT do sudo apt-get update as stated in the ros tutorials. Pal uses its own version of the librairies and packages and doing an update might break compatibility ! If packages are missing, look with `sudo apt search missing_package` and search for the pal repositories (either `pal-robotics/` or `pal-robotics-forks/`). /!\
