# ROSCar_on_PYNQ
----
Pynq Car using xilinx PYNQ-zu: An ROS2 based Mobile Robot Platform


## Aim

## Environment setting
The whole project is running on the PS side.

OS: Linux UBUNTU 22 \
ROS2 distro: rolling

To run this project, first clone this project by:
> $ git clone git@github.com:s095339/ROSCar_on_PYNQ.git

After having the repo successfully installed, create a new branch to protect the main branch

> $ git branch "your branch name"

> $ git checkout "your branch name"

Due to some environment issue, we need to log in as a root user to build and run this repo  

> $ sudo -E -s

And navigate to your ws dir again
> $ cd /home/xilinx/"your ws name"

Copy the following cmd in the terminal
``` sh =
source /opt/ros/rolling/setup.sh
cd ROSCar_on_PYNQ
source install/setup.sh  
export PYTHONPATH=$PYTHONPATH:/usr/local/share/pynq-venv/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/home/[your_user]/ROSCar_on_PYNQ/include
export PYTHONPATH=$PYTHONPATH:/home/[your_user]/ROSCar_on_PYNQ/hardware
```

Build the project
Please build  each packatge separately
>$ colcon build --packages_select [your package]



