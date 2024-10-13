# ROSCon 2024 ros2_control workshop

## Faulty JTC

A variant of the Joint Trajectory Controller which has a service where one can trigger it to fault in the next `update()` call. This failure injection method can be used for instance to demonstrate fallback controllers.

In a terminal:
```
ros2 launch workshop_controllers fallback_controllers.xml.launch
```

In a new terminal try a combination of the following commands to explore, keep an eye on the prints from the first terminal too!
```
ros2 control list_controllers
ros2 service call /faulty_arm_controller/set_fault example_interfaces/srv/SetBool data:\ true\
ros2 control switch_controllers --activate faulty_arm_controller
```
