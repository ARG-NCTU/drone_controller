# drone_controller
A controller-related package for drone
* You can launch multi-controller (control_linear_x.launch, control_linear_y.launch, control_linear_z.launch) within this single launch file:
* And there is a ros node called <fusion_controller.cpp> which will output 3-D twist to mavros.
```
roslaunch drone_controller fusion_control.launch
```
## Params
* There are several params in the controller:
  * <rate>: Define control rate.
  * <P_param>: Define the value of P control.
  * <I_param>: Define the value of I control.
  * <D_param>: Define the value of D control.
  * < margin >: Define the margin where controller should be terminated.

