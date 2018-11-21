unreal_cv_ros is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [unreal engine](https://www.unrealengine.com/en-US/what-is-unreal-engine-4) game. The node-game communcation is carried out utilizing the [unrealcv](https://github.com/unrealcv/unrealcv) computer vision plugin for unreal engine 4 (UE4).

# Table of Contents
**ROS nodes**
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)

**Setting up Unreal**
* [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)
* [Creating UE4 worlds](#Creating-UE4-worlds)
* [Custom collision radius](#Custom-collision-radius)

**Examples**
* [Run in test mode](#Run-in-test-mode)
* [Run in standard mode](#Run-in-standard-mode)

**[Troubleshooting](#Troubleshooting)**

## Dependencies
What should all be added here? The unreal_ros_client node depends on the unrealcv python library `pip install unrealcv`.

# ROS nodes
## unreal_ros_client
This node manages the unrealcv client and the connection with a running UE4 game. It sets the MAV position and orientation within the unreal game and produces the images and camera calibration used by a 3D sensor model.

### Parameters
* **mode** In which mode the client is operated. Currently implemented are:
  * **test** Navigate the MAV manually in the unreal game. The client will periodically publish the sensor data.
  * **standard** The camera pose is set based on the `/odometry` topic and images are taken. Uses the default unrealcv plugin, operates at ~1 Hz.
  * **fast** Like standard, but requires the additional unrealcv command (See [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)). Uses `unreal_cv_ros.msg/UeSensorRawFast` instead of `unreal_cv_ros.msg/UeSensorRaw`, which contains binary image data. Operates at ~3 Hz.
  * **fast2** (Hopefully coming soon) Like fast but requires the modified unrealcv plugin [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup).
  
  Default is 'standard'.
* **collision_on** Set to true to check for collision in the unreal game. Set to false to set the camera anyway. May result in rendering artifacts if the camera overlaps with objects. Default is true.
* **collision_tol** Distance threshold for collision detection in unreal units (default is cm). Will trigger a collision warning if the requested and realized position are further away than the threshold. Default is 10.
* **Hint:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. Its path is displayed when launching the unreal_ros_client node.

### Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the mav pose w.r.t. its positition at the connection of the client. Only appears if test is false.

### Output Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw(Fast)`. The output of the in-game image capture, containing a color and a depth image as well as a camera_info with the camera intrinsics.


## sensor_model
This node converts the UE game output into a pointcloud for further processing and artificially simulates the behaviour of a 3D sensor (e.g. a stereo reconstruction pipeline).

### Parameters
* **mode** The operation mode of the client to align data processing.
* **model_type** Which sensor to simulate. Currently implemented are: 
  * **ground_truth:** Produces the ground truth pointcloud without additional noise. 
  
  Default is 'ground_truth'.

### Input Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw(Fast)`. Output of the unreal_ros_client that is to be further processed.

## Output Topics
* **ue_sensor_out** of type `sensor_msgs.msg/PointCloud2`. Result after applying the simulated sensing pipeline.


# Setting up Unreal

## Unrealcv Plugin Setup
### Standard Plugin
For the modes 'test' and 'standard' the default unrealcv plugin is sufficient. Install it to the project or engine as suggested on their [website](http://docs.unrealcv.org/en/master/plugin/install.html). This repo was tested on unrealcv v0.3.10.

### Adding the 'vget /uecvros/full' command
This additional command is needed for the 'fast' mode.
* **Compile it yourself**  Create an unreal engine development environment as is explained [here](http://docs.unrealcv.org/en/master/plugin/develop.html). Change the source code of the unrealcv plugin in the project (eg. `UnrealProjects/playground/Plugins/UnrealCV/Source/UnrealCV/Private/Commands`) to include the new command:
In `CameraHandler.h` add the declaration:
```c++
/** Run a step of the uecvros routine */
FExecStatus UecvrosFull(const TArray<FString>& Args);
```
  In `CameraHandler.cpp` add the command dispatcher and function body (Copy paste the code from content/CommandCode.cpp). Afterwards build the project as explained in the unrealcv docs. The compiled plugin can thereafter be copied to other projects or the engine.

* **command reference** Take images and then request the next position. This leaves time for the engine to finish movement before the next images are requested.
  - Syntax: `vget /uecvros/full X, Y, Z, p, y, r, tol, X_old, Y_old, Z_old`. All argumentes are floats and need to be set. 
  - Input: 
    - (X, Y, Z): New target position in unreal units.
    - (p, y, r): New target orientation in degrees (and unreal coordinate system).
    - tol: Distance threshold for collision detection. Set to -1 to turn collision off.
    - (X, Y, Z)\_old: Previous target position against which collision is tested.
  - Output: 
    - In case of collision returns a Collision warning. 
    - Otherwise returns binary image data, where the first half representes the color image and the second half the depth image, both as npy arrays.
  
* **Compiled Plugin** TODO: We can make this available somewhere I think.

### Ros node inside the plugin
Will add this section as soon as it really works.

## Creating UE4 worlds
In order to easily create unreal_cv_ros compatible worlds UE4 worlds:
* Install and use unreal engine editor **4.16** (for compatibility with unrealcv).
* Make sure the unrealcv plugin is installed **and** activated in the current project (Edit > Plugins > Science > Unreal CV, see [unrealcv docs](http://docs.unrealcv.org/en/master/plugin/install.html)).
* Set the player to a spectator with collision type: World Settings > Game Mode > Selected GameMode > Default Pawn Class := DefaultPawn.

## Custom collision radius
The default collision for the *DefaultPawn* is a sphere of radius 35cm. For custom collision (radius), you need to create your own pawn blueprint (with DefaultPawn as base class). 'Easy' way to create a pawn of custom collision radius:
1. In the Modes window, search for 'DefaultPawn' and create an instance (drag and drop into the game world).
2. Select the pawn instance and click Blueprints > Convert selected actor to blueprint class...
3. Save the new class, e.g. in the content/blueprints folder as myDefaultPawn.
4. The class should now open in the blueprint editor, where its components can be edited.
5. To change the radius elect the 'CollisionComponent', and under Details > Shape > SphereRadius := myValue. Notice that too short radii can allow the camera to enter certain objects, creating graphical artifacts.
6. Save the blueprint. Set the World Settings > Game Mode > Selected GameMode > Default Pawn Class := myDefaultPawn

# Examples
## Run in test mode
To illustrate the pipeline we run the unreal_ros_client in test mode with ground_truth as our sensor model. Please download the [RealisticRendering](http://docs.unrealcv.org/en/master/reference/model_zoo.html#rr) game binary and launch the game. In a command window type `roslaunch unreal_cv_ros/launch/test_pointcloud.launch` to start the pipeline and wait until the connection is setup (takes few seconds). You can now navigate the drone inside the game using the mouse and W-A-S-D keys while a rviz window displayes the produced ground truth pointclouds as well as the MAV pose in the unreal world frame.

**Note:** Since the taking of images and read-out of the unreal-pose is done sequentially, fast movement in the game may result in the frames not being well aligned.

## Run in standard mode
Setup and run the RealisticRendering demo as in the test example. In a command window type `roslaunch unreal_cv_ros/launch/full_routine.launch` to start the pipeline. A MAV (with its physics and a MPC controller) is simulated in gazebo. It moves to a target pose, which is specified at the end of the launch script. During movement, it will collect pointcloud images from unreal and integrate it into a voxblox mesh. The resulting mesh is displayed in a RVIZ window.

**Note:** During simulation, the unreal game still takes commands from the user. Make sure to tab out of the game so that the user input does not interferewith the simulation setup. 

# Troubleshooting
1. Error addressing the unrealcv client. Try restarting the game.
   - Make sure that only a single unreal game **or** editor is running. When using the editor, the unrealcv plugin is loaded already during startup (without the game itself running!). Since unrealcv per default connects to `localhost:9000` the server will be blocked if any other instance of it is running.