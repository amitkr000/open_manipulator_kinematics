# open_manipulator_kinematics
> A ROS Package to calculate and Publish the Forward and Inverse kinematics of the OpenManipulator-X robotic arm on the specified ros-topic.

The Forward and Inverse kinematic code of openManipulator-x robot arms.
<p>The code is developed during my college assignment.</p>

<img src="https://emanual.robotis.com/assets/images/platform/openmanipulator_x/OpenManipulator_Chain_spec_side.png" height="50%" width="50%" alt="Open_Manipulator_X" title="OpenManipulator-X Dimensions">

**OpenManipulator official Documentation :** https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/

---
## ROS Package Installation

```
cd ~/catkin_ws/src

# retrieve the sources
git clone -b noetic https://github.com/amitkr000/open_manipulator_kinematics.git

cd ~/catkin_ws

# building the package
catkin_make

# activate this workspace
source ~/catkin_ws/devel/setup.bash

Note: If your workspace's name differs, replace the "catkin_ws" with your workspace name.
```
> [!NOTE]
> Both Forward Kinematics and Inverse Kinematics are implemented in the same node, only the topics on which the joint angles and gripper pose are publishing and subscribing are different and more details is clearified below.

***
## Forward Kinematics

> [!CAUTION]
> Source the workspace, where you have cloned the package(open_manipulator_kinematics).
```
# activate this workspace
source ~/catkin_ws/devel/setup.bash

Note: If your workspace's name differs, replace the "catkin_ws" with your workspace name.

rosrun open_manipulator_kinematics kinematic_node
```
Now, the 'kinematic_node' ros node will start and publish the Gripper Pose(**Forward Kinematics Result**) on topic '/omx_kinematics/gripper_kinematic'.

> [!Note]
> But first you have Publish the current joint angles of each joints on topic 'omx_kinematics/joint_angles'.

<br/>

> [!IMPORTANT]
> Publish Forward Kinematics Result on Topic - **'/omx_kinematics/gripper_kinematic'**. <br/>
> Subscribe Joint angle on Topic - **'omx_kinematics/joint_angles'**.

***
## Inverse Kinematics

> [!CAUTION]
> Source the workspace, where you have cloned the package(open_manipulator_kinematics).
```
# activate this workspace
source ~/catkin_ws/devel/setup.bash

Note: If your workspace's name differs, replace the "catkin_ws" with your workspace name.

rosrun open_manipulator_kinematics kinematic_node
```
Now, the 'kinematic_node' ros node will start and publish the Joint state(**Inverse Kinematics Result**) on topic '/omx_kinematics/joint_state'.

> [!Note]
> But first you have Publish the current gripper pose on topic 'omx_kinematics/gripper_pose'.

<br/>

> [!IMPORTANT]
> Publish Inverse Kinematics Result on Topic - **'/omx_kinematics/joint_state'**. <br/>
> Subscribe gripper pose on Topic - **'omx_kinematics/gripper_pose'**.


