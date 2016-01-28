#include <joint_state_recorder/JointStateRecorder.h>

namespace joint_state_recorder
{

    JointStateRecorder::JointStateRecorder(const ros::NodeHandle& nh) :
            nodeHandle(nh), maxPoints(10000), hasData(false)
    {

    }

    JointStateRecorder::~JointStateRecorder()
    {

    }

    void JointStateRecorder::Subscribe(const std::string& name)
    {
        topicName = name;
        jointSubscriber = nodeHandle.subscribe(name, 10, &JointStateRecorder::OnJointStateMsg, this);
    }

    void JointStateRecorder::InsertState(const std::string& name, const double& value, const ros::Time& time)
    {
        if (trajectories.find(name) == trajectories.end())
        {
           JointTrajectory newTraj;
           newTraj.jointName = name;

           trajectories.insert(std::make_pair(name, newTraj));
        }

        JointTrajectory& traj = trajectories[name];

        TrajPoint newPoint;
        newPoint.position = value;
        newPoint.timestamp = time;
        traj.traj.push_back(newPoint);

        if (traj.traj.size() >= maxPoints)
        {
          traj.traj.erase(traj.traj.begin());
        }
    }

    void JointStateRecorder::OnJointStateMsg(const sensor_msgs::JointStateConstPtr& msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            const std::string& name = msg->name.at(i);
            if (trajectories.find(name) == trajectories.end())
            {
                JointTrajectory newTraj;
                newTraj.jointName = name;

                trajectories.insert(std::make_pair(name, newTraj));
            }

            JointTrajectory& traj = trajectories[name];

            TrajPoint newPoint;

            if (msg->effort.size() > 0)
                newPoint.effort = msg->effort.at(i);

            if (msg->position.size() > 0)
                newPoint.position = msg->position.at(i);

            if (msg->velocity.size() > 0)
                newPoint.velocity = msg->velocity.at(i);

            newPoint.timestamp = msg->header.stamp;
            traj.traj.push_back(newPoint);

            if (traj.traj.size() >= maxPoints)
            {
                traj.traj.erase(traj.traj.begin());
            }
        }

        hasData = true;
    }

    sensor_msgs::JointState JointStateRecorder::InterpState(const ros::Time& timestamp)
    {
        sensor_msgs::JointState state;
        state.header.stamp = timestamp;

        for (std::map<std::string, JointTrajectory>::const_iterator it = trajectories.begin();
                it != trajectories.end(); it++)
        {
            state.name.push_back(it->first);
            TrajPoint pt = InterpPoint(it->second, timestamp);
            state.effort.push_back(pt.effort);
            state.position.push_back(pt.position);
            state.velocity.push_back(pt.velocity);
        }
        return state;
    }

    JointStateRecorder::TrajPoint JointStateRecorder::InterpPoint(const JointTrajectory& traj, const ros::Time& timestamp)
    {
        double t = timestamp.toSec();

        TrajPoint pt;
        pt.timestamp = timestamp;

        if (traj.traj.size() == 0)
        {
            return pt;
        }

        if (traj.traj.size() == 1)
        {
            return traj.traj.at(0);
        }

        for (size_t i = 0; i < traj.traj.size() - 1; i++)
        {
            const TrajPoint& pi = traj.traj.at(i);
            const TrajPoint& pi1 = traj.traj.at(i + 1);
            double ti = pi.timestamp.toSec();
            double ti1 = pi1.timestamp.toSec();

            if (ti <= t && ti1 >= t)
            {
                double interp = 1.0 - (ti1 - t) / (ti1 - ti);

                pt.effort = (1.0 - interp) * pi.effort + interp * pi1.effort;
                pt.position = (1.0 - interp) * pi.position + interp * pi1.position;
                pt.velocity = (1.0 - interp) * pi.velocity + interp * pi1.velocity;
                return pt;
            }
        }

        return traj.traj.at(traj.traj.size() - 1);
    }

} /* namespace joint_state_recorder */

