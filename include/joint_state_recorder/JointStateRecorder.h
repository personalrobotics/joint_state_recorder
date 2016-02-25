
#ifndef JOINTSTATERECORDER_H_
#define JOINTSTATERECORDER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>
#include <map>

namespace joint_state_recorder
{
    class JointStateRecorder
    {
        public:
            struct TrajPoint
            {
                double position;
                double velocity;
                double effort;
                ros::Time timestamp;
            };

            struct JointTrajectory
            {
                std::string jointName;
                std::vector<TrajPoint> traj;
            };
            JointStateRecorder(const ros::NodeHandle& nh);
            JointStateRecorder(const ros::NodeHandle& nh, const std::map<std::string, bool>& continuousMap);
            virtual ~JointStateRecorder();

            void Subscribe(const std::string& name);
            void OnJointStateMsg(const sensor_msgs::JointStateConstPtr& msg);
            // Get the joint state message at the given time. Interpolates
            // between joint messages from other times.
            sensor_msgs::JointState InterpState(const ros::Time& timestamp);
            sensor_msgs::JointState GetLatest();
            inline bool HasData() { return hasData; }

        protected:
            TrajPoint InterpPoint(const JointTrajectory& traj, const ros::Time& timestamp);
            TrajPoint GetLatest(const JointTrajectory& traj);
            void InsertState(const std::string& name, const double& value, const ros::Time& time);
            ros::NodeHandle nodeHandle;
            std::map<std::string, JointTrajectory> trajectories;
            std::string topicName;
            ros::Subscriber jointSubscriber;
            size_t maxPoints;
            bool hasData;
            std::map<std::string, bool> continuousMap;
    };

} /* namespace joint_state_recorder */

#endif /* JOINTSTATERECORDER_H_ */

