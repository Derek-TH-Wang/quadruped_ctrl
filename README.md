# quadruped_robot

### MIT mini cheetah simulation in pybullet
MIT mini cheetah use qt simulation env, we extract the algorithm and do the simulation use ros and pybullet env.
This is easy to deploy the system into different custom robot or plantform, and also easy to learn the algorithm.

<img src="https://github.com/Derek-TH-Wang/quadruped_ctrl/blob/master/quadruped_balance.gif" alt="show" />

### System requirements:
Ubuntu 18.04, ROS Mellodic  

### Dependency:
use Logitech gamepad to control robot  
```
git clone https://github.com/Derek-TH-Wang/gamepad_ctrl.git
```

### Running:
run the controller in simulator:  
```
roslaunch quadruped_ctrl quadruped_ctrl.launch
```
you can switch the gait type:  
```
rosservice call /gait_type "cmd: 1"
```

gait type:
```
0:trot
1:bunding
2:pronking
3:random
4:standing
5:trotRunning
6:random2
7:galloping
8:pacing
9:trot (same as 0)
10:walking
11:walking2
```

