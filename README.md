# quadruped_robot

### MIT mini cheetah simulation in pybullet
MIT mini cheetah use qt simulation env, we extract the algorithm and do the simulation use ros and pybullet env.
This is easy to deploy the system into different custom robot or plantform, and also easy to learn the algorithm.

### System requirements:
Ubuntu 18.04, ROS Mellodic  

### Dependency:
```
git clone https://github.com/Derek-TH-Wang/cheetah.git
```

### Running:
```
rosrun cheetah QuadListener.py
roslaunch quadruped_robot quadruped_robot.launch
```
first set to stand up:  
```
rosservice call /ctrl_mode "cmd: 1"
```
then, set to tort gait:  
```
rosservice call /ctrl_mode "cmd: 4"
```