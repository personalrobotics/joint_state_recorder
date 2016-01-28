# joint_state_recorder
Record ROS JointState messages and index them by time.

Usage:

```
#include <joint_state_recorder/JointStateRecorder.h>
#include <sensor_msgs/JointState.h>

using namespace joint_state_recorder;

// Create a recorder and subscribe to a message
JointStateRecorder recorder(nodeHandle);
recorder.Subscribe("your_topic_name");

// Do some processing ...
ros::spinOnce();
//...

// Get the joint state at the given time.
sensor_msgs::JointState currentJointState = recorder.InterpState(ros::Time::now());

```

This allows the user to synchronize joint state messages with other messages, to determine what the state of the robot was at a given time (similar to `tf`). States are linearly interpolated. 
