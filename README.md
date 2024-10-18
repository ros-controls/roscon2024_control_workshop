# ROSCon 2024 ros2_control workshop

## Setup this repository

1. Clone the repository to your existing ROS 2 humble+ workspace. Either local or docker based workspace.

1. Import dependencies that need compilation from source using the command:
   ```
   vcs import --input roscon2024_control_workshop/roscon2024_control_workshop.repos .
   ```

1. Install all dependencies using `rosdep`:
   ```
   rosdep install -y -i --from-paths .
   ```

1. Compile the workspace using `colcon`, e.g., `colcon build` executed in the root of the workspace (where `src` folder is located).


## Task 1: Controller Chaining

This task shows how a controllers are chained with ros2_control.

#### Scenario
Using PAL Tiago robot we want to reuse filtered data from the range sensor in the `RangeSensorBroadcaster` and in `DiffDriveController`.
We can of course filter those data separately in each controller, but than is is hard to make sure that filters' parameters are synchronized all the time.
Therefore, we introduce a `ChainedFilter` controller that filters the data and provides state interfaces for other controllers to read those data.
Of course, the controllers using it has to have reference interfaces to be able to connect to other controllers. In this example `DiffDriveController` is not chainable and we have to make it such.

Also, we want to process the velocity values from `DiffDriveController` with PID before sending those the hardware.
Therefore, `DiffDriveController` has to be enables to write to the interface with different prefix then reading from.

#### Running the example

1. In a terminal where your workspace is sourced start launch file:
   ```
   ros2 launch workshop_bringup tiago_chaining.xml.launch
   ```


## Task 2: Fallback Controllers

There is a new feature in ros2_control that support automatic replacement of a controller that might fail of throw exception during `update` method.
Use of this feature is recommended on all robotic platforms that are inherently unstable, e.g., walking robots.

One other example is using of robotic arms in torque mode.
To avoid sacking if something is not OK with the controller, we can now, for example, use JTC as *fallback controller* to our custom controller.
The controllers from ros2_controllers repository are generally a good choice for fallback controllers as they are tested very well, but might have limited functionalities compared to your fancy controller.

**As starting of *fallback* controllers might be disturbed by the configuration of other controllers, you HAVE TO always test *fallback* strategies before starting it on the hardware!**


#### Scenario
In this task we are running a `Faulty JTC` that can "crash" on the next `update()` call after a special service is called and it is replaced with the standard JTC.
This method for failure injection is used only for demonstration, but it is also useful to use similar or the same approach when testing *fallback* setup.


A variant of the Joint Trajectory Controller which has a service where one can trigger it to fault in the next `update()` call. This failure injection method can be used for instance to demonstrate fallback controllers.

#### What to focus on
Make sure to understand the configuration of the `fallback` controller in the [fallback_controllers.yaml](./workshop_bringup/config/fallback_controllers.yaml).


#### Running the example

1. In a terminal where your workspace is sourced start launch file:
   ```
   ros2 launch workshop_bringup tiago_fallback.launch.xml
   ```

2. Now list the controllers in another terminal also where your workspace is sourced:
   ```
   ros2 control list_controllers
   ```

3. Call service to provoke error of the `Faulty JTC`:
   ```
   ros2 service call /faulty_arm_controller/set_fault example_interfaces/srv/SetBool data:\ true\
   ```

4. Now list the controllers in another terminal also where your workspace is sourced:
   ```
   ros2 control list_controllers
   ```

5. Activate again the `Faulty JTC`:
   ```
   ros2 control switch_controllers --activate faulty_arm_controller
   ```


# Task 3: Async Controllers


# Task 4: Multi-robot scalable parameter handling

This task demonstrates how to integrate multiple robots under different controller manager instances and how to handle parameters for better scalability of controller's configuration.

#### Scenario
Using PAL TIAGo robot we are going to repeat the same controller chaining as in deploying on a fleet of robots. The idea is to make the setup work on different robots working on different namespaced controller_manager. 

#### What to focus on
Make sure to understand the controller configuration present in the [chaining_controllers_tiago1.yaml](./workshop_bringup/config/chaining_controllers_tiago1.yaml) and [chaining_controllers_generic.yaml](./workshop_bringup/config/chaining_controllers_generic.yaml).


#### Running the example

1. In a terminal where your workspace is sourced start launch file:
   ```
   ros2 launch workshop_bringup tiago_chaining.launch.launch.xml namespace:=tiago1
   ```

   You should see that the different controllers are failing as the initial setup defined in the [chaining_controllers.yaml](./workshop_bringup/config/chaining_controllers.yaml) is not enough to work with the different controller managers setup

2. In a terminal where your workspace is sourced start launch file:
   ```
   ros2 launch workshop_bringup tiago_chaining.launch.launch.xml namespace:=tiago1 config_file:=chaining_controllers_tiago1
   ```

   Now, launching the [chaining_controllers_tiago1.yaml](./workshop_bringup/config/chaining_controllers_tiago1.yaml) configuration file works because all the parameters are namespaced to the robot's serial number and it works. But!!!! you need to hardcode the namespacing in each and every robot's configuration file. If this works for you, then :thumbsup:. If not, the point 3 is for you!

   ```
   ros2 launch workshop_bringup tiago_chaining.launch.launch.xml namespace:=tiago2 config_file:=chaining_controllers_tiago1
   ```

   launching the above command should fail for you as it cannot find the corresponding parameters

3. In a terminal where your workspace is sourced start launch file:
   ```
   ros2 launch workshop_bringup tiago_chaining.launch.launch.xml namespace:=tiago1 config_file:=chaining_controllers_generic
   ```
   ```
   ros2 launch workshop_bringup tiago_chaining.launch.launch.xml namespace:=tiago2 config_file:=chaining_controllers_generic
   ```

4. Now list the controllers in another terminal also where your workspace is sourced:
   ```
   ros2 control list_controllers --controller-manager /tiago1/controller_manager
   ros2 control list_controllers --controller-manager /tiago2/controller_manager
   ```

   You should see the same set of the controllers loaded in both the controller managers

   [chaining_controllers_generic.yaml](./workshop_bringup/config/chaining_controllers_generic.yaml) has the controller's parameters set with the wildcard entries and this makes it scalable when deploying on a fleet of robots
