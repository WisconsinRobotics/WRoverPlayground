# WRoverPlayground

Welcome to WRoverPlayground! These learning modules will help you become somewhat proficient in using the Robot Operating System (ROS). Each module builds on the previous ones. When you are finished, you will have built software for a robot named WRunt. This robot will be able to autonomously navigate through a course of infrared (IR) beacons from a starting location. *Add a picture of the course for illustrative purposes. A GIF of WRunt completing a course would be even better*

Your software will allow WRunt to detect beacons and navigate between them. The only instructions that can be given to WRunt during operation are to tell it to go to the next beacon in the course. Once a beacon is reached WRunt will let us know that it has arrived by lighting up its status light. Below are more detailed descriptions of how WRunt will be controlled using ROS. Don't worry if this doesn't make sense now! The modules will walk you through this step by step.

## WRunt specifications

### Drive system

WRunt will have two drive wheels. The drive system used is called a tank drive. This means that each side of the robot is controlled independently. When both sides are commanded to go forward, the robot will move forward. When one side is commanded forward and the other backward, the robot spins. You can get creative with this and put one side at a slightly lower power to move the robot on a curve.

### Infrared (IR) sensor

The IR sensor detect IR radiation from the closest beacon and gives you the heading of the closest beacon relative to the robot. You'll need to use this information to figure out where to drive WRunt.

### Status light

The status light lights up when WRunt has reached the target. This allows observers to know when WRunt knows it has reached its target. During a competition setting, this lets the judges know that the robot actually completed the task and did not just accidentally get to the right spot.

### Continue to next beacon signal

Once WRunt has reached the first beacon, you will need to give it a manual signal that it can move on to the next beacon. In real life, this signal would be transmitted over a network connection. For our pursposes, it will be a button on a Graphical User Interface (GUI).

## WRunt behavior flowchart

1. WRunt determines the closest beacon that hasn't been visited yet
2. WRunt move to and reaches the target beacon
3. WRunt marks the beacon as visited, changes its own status light, and awaits a certain number of seconds
4. Repeat steps 1-3 until all beacons have been reached

## How To Open The Project

This project uses Docker to efficiently containerize the workspace and make cross-platform development easier.  Find your host platform below to see how to start development:

### Linux

#### Tools:

* [git](https://git-scm.com/)
* [VS Code](https://code.visualstudio.com/)
* [Docker Engine](https://docs.docker.com/engine/)

#### Instructions:

These instructions are written for Ubuntu, steps may vary if using a different distribution.

1. Install git: `sudo apt install git`
2. Install VS Code: https://code.visualstudio.com/docs/setup/linux
3. Install Docker: https://docs.docker.com/engine/install/ubuntu/
4. Set up SSH keys
5. Clone the repository
6. Open the repository with VSCode
7. Reopen the folder in container

#### Troubleshooting:

### Windows

#### Tools:

* [git](https://git-scm.com/)
* [VS Code](https://code.visualstudio.com/)
* [WSL 2](https://www.omgubuntu.co.uk/how-to-install-wsl2-on-windows-10)
* [Docker Desktop](https://www.docker.com/products/docker-desktop/)

#### Instructions:

1. Install tools
2. Set up SSH keys on Windows and in WSL
3. Clone the repository inside WSL
4. Open the repository with VSCode
5. Launch Docker
6. Reopen the folder in container

#### Troubleshooting:

Unable to execute git push or pull in container: https://superuser.com/questions/1726204/get-agent-identities-ssh-agent-bind-hostkey-communication-with-agent-failed

### Mac

#### Tools:

* [git](https://git-scm.com/)
* [VS Code](https://code.visualstudio.com/)
* [XQuartz](https://www.xquartz.org/)
* [Docker Desktop](https://www.docker.com/products/docker-desktop/)

#### Instructions:

1. Install tools (beware that install XQuartz requires a logout/login, so save any local changes on your computer before installing XQuartz)
2. Set up SSH keys
3. Clone the repository
4. Open the repository with VSCode
5. Launch Docker Desktop
6. In VSCode's command palette (F1), run `Dev Containers: Open Folder in Container`

To ensure docker works with graphing tools, run `roscore` in VSCode's terminal and `rqt_plot` in another terminal. Follow instructions below if a new window fails to start and the terminal outputs something similar to:
```
could not connect to display 
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
```
1. Start XQuartz with `open -a XQuartz`
2. Under the security section of settings, check "Allow connections from network clients"
3. Restart your Mac and start XQuartz again
4. Ensure that XQuartz is running correctly with `ps aux | grep Xquartz`. There should be a line that is similar to `/opt/X11/bin/Xquartz :0 -listen tcp`. Make sure it is not `–nolisten tcp`.
5. Allow X11 forwarding with `xhost +` to allow any client to connect or `xhost +localhost` to limit clients to localhost. Note that `xhost +` is not a persistent setting, so you would have to rerun it every time you restart X11
6. Go back to your docker environment and try running `rqt_plot` again, this time the display should show. 

#### Troubleshooting:

Unable to display `rqt` tools or other graphics: https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088
