# rmd_motor_controller

rmd motor controller with ros

\#run

first you need to start the motion controller, this node can generate the velovity base on your local planner

```
rosrun motion_controller motion_controller_node
```

then you can start the rmd motor controll node

```
rosrun rmd_motor_controller_frontleft rmd_motor_controller_frontleft_node
```

