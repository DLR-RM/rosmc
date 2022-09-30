# How to use ROSMC

## Step 1. Implement high-level skills
First, implement high-level skills with RAFCON referring to [mission_control_rafcon_statemachines](https://github.com/DLR-RM/mission_control_rafcon_statemachines.git).

## Step 2. Configure ROSMC
Several config files are required to run ROSMC with your robots.
You can find examples under [rosmc_mission_server/configs](rosmc_mission_server/configs).
### agent_actionlib
**'agent_actionlib'** defines which dynamic agents exist in your mission and which high-level skills they can execute.
This file has to have the following format:

```yaml
agent1_name: 
  skill_root_dir: path_to_rafcon_library_of_high_level_skills
  name: LIBRARY_NAME 
agent2_name:
  skill_root_dir: path_to_rafcon_library_of_high_level_skills
  name: LIBRARY_NAME
<...>
```

### parameter_group
Input parameters of high-level skills can be grouped in the GUIs.
Each **'LIBRARY_NAME'** used in the **'agent_actionlib'** config file should have one such config file named **'LIBRARY_NAME.yaml'**.
These confg files should be stored in a `parameter_group` directory.
Please refer to the example file at [rosmc_mission_server/configs/parameter_group/turtle_test_skills.yaml](rosmc_mission_server/configs/parameter_group/turtle_test_skills.yaml) for the format and the documentation.

### int_markers
We can use interactive markers to specify values of parameters of high-level skills.

There are two types of interactive markers: *agent* and *task*.
*Agent* markers represent agents used in the mission.
The one that represents a robot which can be commanded by ROSMC is called a *dyanmic agent*.
On the other hand, the one that represents a pre-known object (such as boxes to be manipulated by robots) is called a *static agent*.
*Task* markers represent POIs and ROIs that users can create and move/resize.

#### Servers
You need to customize the agent interactive marker servers as follows:
- *Dynamic agent* marker servers.
  The basic class is provided in [rosmc_agents/src/rosmc_agents/dynamic_agent_int_marker_publisher.py](rosmc_agents/src/rosmc_agents/dynamic_agent_int_marker_publisher.py).
  An example of an inherited class for turtlesim is provided in [rosmc_agents/src/turtle_int_marker_publisher.py](rosmc_agents/src/turtle_int_marker_publisher.py)
- *Static agent* marker servers.
  The basic class is provided in [rosmc_agents/src/rosmc_agents/static_agent_int_marker_publisher.py](rosmc_agents/src/rosmc_agents/static_agent_int_marker_publisher.py).
  You can inherit the class and custamize it similar to the dynamic agent markers.

The followings are optional; if you want, you can extend the task marker servers and Rviz plugins.
- Your custom *task* marker servers.
  We provide three basic types of task marker servers: 2D position, 2D pose, and 2D area in [rosmc_task_marker_server/src/rosmc_task_marker_server](rosmc_task_marker_server/src/rosmc_task_marker_server).
  If you need your custom task markers, inherit one of these classes 
- Your custom *task* marker tools.
  To make it possible to add your custom *task* marker from Rviz, you need to implement the corresponding Rviz tool plugin.
  We provide the plugins for the three basic task markers under [rosmc_task_marker_tools](rosmc_task_marker_tools).

#### Config files
Using config files, we define which input parameter of which skill can be assigned by which type of interactive markers.

There are two types of this config file: `common.yaml` and `LIBRARY_NAME.yaml`.
`common.yaml` configures which interactive marker server serves which type of interactive marker (*agent*, *task*, and so on).
`LIBRARY_NAME.yaml` configures the correspondence between parameters and interactive markers.

See [int_markers](rosmc_mission_server/configs/int_markers) as an example.


## Step 3. Start processes
* Set the following rosparams:
  * **'use_interactive_markers'** (bool): true if you use interactive markers
  * **'frame_id'** (str): tf frame_id of the global coordinate system shared with all the agents
  * **'agent_actionlib_config_path'** (str): path to the agent_actionlib config file
  * **'parameter_group_config_folder'** (str): path to the directory which contains parameter_group config files
  * **'int_marker_config_folder'** (str): path to the directory which contains int_marker config files
  * **'task_marker_offset_from_ground'** (float): vertical (z-axis) offset of the task markers from the global coordinate system
  * **'task_marker_scale'** (float): scale of the task markers (default: 1.0)
* Start the following processes:
  * Mission server: `rosrun rosmc_mission_server mission_server.py`
  * Task marker servers
    * 2D position: `rosrun rosmc_task_marker_server task_marker_2d_position_server_node.py`
    * 2D pose: `rosrun rosmc_task_marker_server task_marker_2d_pose_server_node.py`
    * 2D area: `rosrun rosmc_task_marker_server task_marker_2d_area_server_node.py`
    * ...and your custom task marker servers if needed
  * Agent marker servers -> depends on your mission. For the turtlesim demo, `rosrun rosmc_agents turtles_int_marker_publisher.py`
  * GUIs
    * Status icon publisher: `rosrun rosmc_status_monitor status_icons_publisher.py`
    * Comand GUI: `rosrun rosmc_command_gui rosmc_command_gui`
    * 3D GUI: `rviz`. You need to add topics of interactive markers and status icons from the display panel on the left.

## Step 4. Create & execute mission

### Basics
You can use the two GUIs (the Rqt-based command GUI and the Rviz) to create, run, and monitor the mission.

#### Create a mission
First, let's add skills to compose a mission.
In the command GUI,
- click the "Add" button at the bottom
- select the skill you want from the tree menu

Or in the Rviz,
- click the agent interactive marker
- select the skill you want from the tree menu

Then in the command GUI, you should be able to see the skill is added to the mission!

Next step is to adapt the parameter values of the skill.
Click the skill and you can see the parameter values on the right-hand side of the command GUI.
You can update the values via the text box.
If a task interactive marker is associated with the skill parametrization, the interactive marker also updates its pose in the Rviz.
You can also update the values via the Rviz.
If you drag and change the pose of the task interactive marker in the Rviz, the values shown in the command GUI is also updated.

Another convenient way of adding skills is to first create a new interactive marker in the Rviz and then use it for a skill.
From the top menu bar of the Rviz, you can add the tool plugin for creating task markers.
For example, you can click the "2D pose task maker" tool and then click inside the 3D view of the Rviz.
This will create a new instance of the 2D pose task marker.
You can click this task marker, which pops up the tree menu.
From the tree menu, you can choose which robot to do which skill for this task marker.
The selected skill is added to the mission for the selected robot.
You can confirm this in the command GUI.

To synchronize the execution of skills by multiple robots, you can create a skill barrier.
Click the skills you want to synchronize over robots by pressing ctrl key.
Then right clicking shows a tree menu, from which you can select "Add synchronization point".
Adding a synchronisation point allows those skills to be triggered at the same moment.

#### Saving/loading a mission
Click the "Save" or "Load" button at the top of the command GUI.
This will take care saving/loading the mission file including all the task markers in the Rviz.

#### Synchonize (upload) a mission to robots
Once you feel good enough with the current mission, you can click the "Sync" button and upload the mission to the robots.
In case the communication failure happens (e.g. becase the robot is suffering from the network loss), the synchronization request is safely declined and the users are notified by a popup window.

The synchronized part of the mission is moved to the upper half of the command GUI.
Only the synchronized part of the mission is executed by the robot autonomously.
The communication between the ground station and the robots only happens when the synchronized part of the mission is updated.
Therefore, even under a huge communication latency, users can keep composing the upcoming part of the mission locally as a non-synchronized part.

#### Run the mission!
Just click the "Run" button!
Please make sure that you have started the mission executor (e.g. RAFCON) per robot, which receives the mission from ROSMC and actually runs the skills.

#### Something went wrong! I want to pause/stop/adapt the mission!
Don't worry, that is one of the nice featuer of ROSMC to allow for simultaneous adaptation of the mission while running.
You can always freely compose/adapt the non-synchronized part of the mission.
To modify the synchronized mission, you need to "unsynchronize" it from the robot to make sure that the robot does not execute it while you edit.
You can also unsynchronize only specific skills you want to modify and keep the rest synchronized to the robots.
