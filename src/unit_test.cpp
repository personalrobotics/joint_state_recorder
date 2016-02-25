/*
 * unit_test.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: mklingen
 */

#include <ros/ros.h>
#include <joint_state_recorder/JointStateRecorder.h>
#include <fstream>

using namespace joint_state_recorder;
using namespace std;
using namespace sensor_msgs;
using namespace ros;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_state_recorder");
    NodeHandle nodeHandle("~");
    map<string, bool> continuous;
    continuous["J1"] = true;
    continuous["J2"] = false;
    JointStateRecorder recorder(nodeHandle, continuous);
    int num_configs = 100;
    float timestep = 0.1;
    float t = 0;
    std::vector<double> j1s;
    std::vector<double> j2s;
    std::vector<double> ts;
    for (int i = 0; i < num_configs; i++)
    {
        ts.push_back(t);
        JointState* joints = new JointState();
        joints->name.push_back("J1");
        joints->name.push_back("J2");
        joints->position.push_back(atan2(sin(t), cos(t)));
        joints->position.push_back(atan2(cos(t), sin(t)));
        joints->header.stamp.fromSec(t);
        j1s.push_back(atan2(sin(t), cos(t)));
        j2s.push_back(atan2(cos(t), sin(t)));
        recorder.OnJointStateMsg(JointStateConstPtr(joints));
        t += timestep;
    }

    float interpTime = 0.01;
    std::vector<double> interp_j1s;
    std::vector<double> interp_j2s;
    std::vector<double> interp_ts;
    for (float t_interp = 0; t_interp < num_configs * timestep; t_interp += interpTime)
    {
        try
        {
            ros::Time time;
            time.fromSec(t_interp);

            JointState interp_state = recorder.InterpState(time);
            interp_j1s.push_back(interp_state.position[0]);
            interp_j2s.push_back(interp_state.position[1]);
            interp_ts.push_back(t_interp);
        }
        catch(invalid_argument& e)
        {

        }

    }

    std::ofstream sampleFile("samples.txt", std::ios::out);

    for (int i = 0; i < num_configs; i++)
    {
        sampleFile << ts.at(i) << " " << j1s.at(i) << " " << j2s.at(i) << std::endl;
    }

    sampleFile.close();

    std::ofstream interpFile("interp.txt", std::ios::out);

    for (size_t i = 0; i < interp_j1s.size(); i++)
    {
        interpFile << interp_ts.at(i) << " " << interp_j1s.at(i) << " " << interp_j2s.at(i) << std::endl;
    }

    interpFile.close();

    return 0;
}

