# joint_state_recorder
Record ROS JointState messages and index them by time.

Usage:

```c++
#include <joint_state_recorder/JointStateRecorder.h>
#include <sensor_msgs/JointState.h>

using namespace joint_state_recorder;

// Create a recorder and subscribe to a message
JointStateRecorder recorder(nodeHandle);
recorder.Subscribe("your_topic_name");

// Do some processing ...
ros::spinOnce();
//...

// Get a timestamp from some other sensor message 
// (like a camera)
ros::Time timestamp = ...

// Get the joint state at the given time.
try
{
  sensor_msgs::JointState recordedJointState = recorder.InterpState(timestamp);
}
catch (std::invalid_argument& e)
{
  // This is thrown whenever the provided time is before or after joint data exists.
  // (similar to tf::ExtrapolationException)
}
```

This allows the user to synchronize joint state messages with other messages, to determine what the state of the robot was at a given time (similar to `tf`). States are linearly interpolated. 

## Continuous Joints

If your robot has continuous (i.e non-limited) joints, you can tell `JointStateRecorder` this information by providing a map from joint name to whether the joint is continuous like this:

```c++
std::map<std::string, bool> continuousJoints;
// The robot has one continuous joint, and one 
// limited joint.
continuousJoints["continuous_joint_1"] = true;
continuousJoints["limited_joint_2"] = false;
//...
JointStateRecorder recorder(nodeHandle, continuousJoints);
```

The continuous joints will be forced into the range of `-PI` to `PI` and will wrap with circular topology. Non-continuous joints will linearly interpolate like any other real number.
