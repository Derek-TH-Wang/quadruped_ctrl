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
run the simulator  
```
rosrun quadruped_ctrl walking_simulation.py
```
first set to stand up:  
```
rosservice call /set_jm "cmd: 0" 
rosservice call /set_jm "cmd: 2" 
```
then, run the controller:  
```
rosrun quadruped_ctrl mainController
```
