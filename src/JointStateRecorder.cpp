#include <joint_state_recorder/JointStateRecorder.h>
#include <math.h>
namespace joint_state_recorder
{

    // Linearly interpolate between A and B.
    template <typename T> T lerp(T a, T b, T interp)
    {
        return (1.0 - interp) * a + interp * b;
    }

    // This is a hack that interpolates two angles A and B
    // by projecting the angles into cartesian space, interpolating
    // there, and then re-projecting back onto the unit circle.
    // It isn't quite correct, but its fast.
    template <typename T> T circLerp(T a, T b, T interp)
    {
        T CS = (1.0 - interp) * cos(a) + interp * cos(b);
        T SN = (1.0 - interp) * sin(a) + interp * sin(b);
        return atan2(SN, CS);
    }

    // This is the proper (but expensive) interpolation on
    // the unit circle.
    template <typename T> T slerp1D(T a, T b, T interp)
    {
        T p1x = cos(a);
        T p1y = sin(a);
        T p2x = cos(b);
        T p2y = sin(b);
        T omega = fabs(acos(p1x * p2x + p1y * p2y));
        T aMult = sin((1.0 - interp) * omega) / sin(omega);
        T bMult = sin(interp * omega) / sin(omega);
        T sX = aMult * p1x + bMult * p2x;
        T sY = aMult * p1y + bMult * p2y;
        T returnVal = atan2(sY, sX);
        return returnVal;
    }


    JointStateRecorder::JointStateRecorder(const ros::NodeHandle& nh) :
            nodeHandle(nh), maxPoints(10000), hasData(false)
    {

    }

    JointStateRecorder::JointStateRecorder(const ros::NodeHandle& nh, const std::map<std::string, bool>& continuousMap_) :
            nodeHandle(nh), maxPoints(10000), hasData(false), continuousMap(continuousMap_)
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

    sensor_msgs::JointState JointStateRecorder::GetLatest()
    {
        sensor_msgs::JointState state;
        state.header.stamp = ros::Time::now();

        for (std::map<std::string, JointTrajectory>::const_iterator it = trajectories.begin();
                it != trajectories.end(); it++)
        {
            state.name.push_back(it->first);
            TrajPoint pt = GetLatest(it->second);
            state.effort.push_back(pt.effort);
            state.position.push_back(pt.position);
            state.velocity.push_back(pt.velocity);
        }
        return state;
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

    JointStateRecorder::TrajPoint JointStateRecorder::GetLatest(const JointTrajectory& traj)
    {
        TrajPoint pt;
        pt.timestamp = ros::Time::now();

        if (traj.traj.size() == 0)
        {
            ROS_ERROR("Trajectory is empty.");
            return pt;
        }

        return traj.traj.at(traj.traj.size() - 1);
    }

    JointStateRecorder::TrajPoint JointStateRecorder::InterpPoint(const JointTrajectory& traj, const ros::Time& timestamp)
    {
        double t = timestamp.toSec();

        TrajPoint pt;
        pt.timestamp = timestamp;

        if (traj.traj.size() == 0)
        {
            ROS_ERROR("Trajectory is empty.");
            return pt;
        }

        if (traj.traj.size() == 1)
        {
            ROS_ERROR("Trajectory has size 1");
            return traj.traj.at(0);
        }

        bool continuous = (continuousMap.find(traj.jointName) != continuousMap.end()) && continuousMap[traj.jointName];

        double minTime = std::numeric_limits<double>::max();
        double maxTime = -std::numeric_limits<double>::max();
        for (size_t i = 0; i < traj.traj.size() - 1; i++)
        {
            const TrajPoint& pi = traj.traj.at(i);
            const TrajPoint& pi1 = traj.traj.at(i + 1);
            double ti = pi.timestamp.toSec();
            double ti1 = pi1.timestamp.toSec();
            if (ti < minTime) minTime = ti;
            if (ti1 > maxTime) maxTime = ti1;
            if (ti <= t && ti1 >= t)
            {
                double interp = 1.0 - (ti1 - t) / (ti1 - ti);

                pt.effort = lerp(pi.effort, pi1.effort, interp);
                pt.position =  continuous ? circLerp(pi.position, pi1.position, interp) : lerp(pi.position, pi1.position, interp);
                pt.velocity =  lerp(pi.velocity, pi1.velocity, interp);
                return pt;
            }
        }

        ROS_ERROR("Could not extrapolate or interpolate state at time %f, min was %f, max was %f", t, minTime, maxTime);
        throw std::invalid_argument("Could not interpolate or extrapolate state");
        return traj.traj.at(traj.traj.size() - 1);
    }

} /* namespace joint_state_recorder */

