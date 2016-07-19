// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "../include/sensor_msgs_JointState.h"
#include <string>
#include <time.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;


int main(int argc, char const *argv[]) {

    Network yarp;
    Node node("/icub_sim/state_publisher");

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return 1;
    }
    std::string robotName=params.find("robot").asString().c_str();
    std::string remotePorts="/";
    remotePorts+=robotName;
    std::string remoteRArmPorts = remotePorts + "/right_arm";
    std::string remoteHeadPorts = remotePorts + "/head";
    std::string remoteTorsoPorts = remotePorts + "/torso";
    std::string remoteLArmPorts = remotePorts + "/left_arm";

    Property options_head;
    options_head.put("device", "remote_controlboard");
    options_head.put("local", "/head/client");   //local port names
    options_head.put("remote", remoteHeadPorts.c_str());
    // create a device
    PolyDriver headDevice(options_head);
    if (!headDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    Property options_torso;
    options_torso.put("device", "remote_controlboard");
    options_torso.put("local", "/torso/client");   //local port names
    options_torso.put("remote", remoteTorsoPorts.c_str());
    // create a device
    PolyDriver torsoDevice(options_torso);
    if (!torsoDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    Property options_right_arm;
    options_right_arm.put("device", "remote_controlboard");
    options_right_arm.put("local", "/right_arm/client");   //local port names
    options_right_arm.put("remote", remoteRArmPorts.c_str());
    // create a device
    PolyDriver rArmDevice(options_right_arm);
    if (!rArmDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }


    Property options_left_arm;
    options_left_arm.put("device", "remote_controlboard");
    options_left_arm.put("local", "/left_arm/client");   //local port names
    options_left_arm.put("remote", remoteLArmPorts.c_str());
    // create a device
    PolyDriver lArmDevice(options_left_arm);
    if (!lArmDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *posHead;
    IPositionControl *posTorso;
    IPositionControl *posRArm;
    IPositionControl *posLArm;
    IEncoders *encsHead;
    IEncoders *encsTorso;
    IEncoders *encsRArm;
    IEncoders *encsLArm;

    torsoDevice.view(posTorso);
    torsoDevice.view(encsTorso);
    rArmDevice.view(posRArm);
    rArmDevice.view(encsRArm);
    lArmDevice.view(posLArm);
    lArmDevice.view(encsLArm);
    headDevice.view(posHead);
    headDevice.view(encsHead);


    int nj = 0;

    posHead->getAxes(&nj);
    Vector encodersHead;
    encodersHead.resize(nj);

    nj = 0;
    posTorso->getAxes(&nj);
    Vector encodersTorso;
    encodersTorso.resize(nj);

    nj = 0;
    posRArm->getAxes(&nj);
    Vector encodersRArm;
    encodersRArm.resize(nj);

    nj = 0;
    posLArm->getAxes(&nj);
    Vector encodersLArm;
    encodersLArm.resize(nj);



    yarp::os::Publisher<sensor_msgs_JointState> joint_pub;
    if (!joint_pub.topic("/joint_states")) {
        std::cerr<< "Failed to create publisher to /chatter\n";
        return -1;
    }

    float degtorad = 0.0174532925;

    sensor_msgs_JointState joint_states;
    struct timespec currentTime;

    joint_states.name.resize(70);
    joint_states.position.resize(70);
    joint_states.velocity.resize(70);

    joint_states.name[0] ="j1";
    joint_states.position[0] = 0.0;
    joint_states.name[1] ="j2";
    joint_states.position[1] = 0.0;
    joint_states.name[2] ="j3";
    joint_states.position[2] = 0.0;
    joint_states.name[3] ="j4";
    joint_states.position[3] = 0.0;
    joint_states.name[4] ="j5";
    joint_states.position[4] = 0.0;
    joint_states.name[5] ="j6";
    joint_states.position[5] = 0.0;
    joint_states.name[6] ="j7";
    joint_states.position[6] = 0.0;
    joint_states.name[7] ="j8";
    joint_states.position[7] = 0.0;
    joint_states.name[8] ="j7s";
    joint_states.position[8] = 0.0;
    joint_states.name[9] ="j8s";
    joint_states.position[9] = 0.0;
    joint_states.name[10] ="raj1";
    joint_states.position[10] = 0.0;
    joint_states.name[11] ="raj2";
    joint_states.position[11] = 0.0;
    joint_states.name[12] ="raj3";
    joint_states.position[12] = 0.0;
    joint_states.name[13] ="raj4";
    joint_states.position[13] = 0.0;
    joint_states.name[14] ="raj5";
    joint_states.position[14] = 0.0;
    joint_states.name[15] ="raj6";
    joint_states.position[15] = 0.0;
    joint_states.name[16] ="laj1";
    joint_states.position[16] = 0.0;
    joint_states.name[17] ="laj2";
    joint_states.position[17] = 0.0;
    joint_states.name[18] ="laj3";
    joint_states.position[18] = 0.0;
    joint_states.name[19] ="laj4";
    joint_states.position[19] = 0.0;
    joint_states.name[20] ="laj5";
    joint_states.position[20] = 0.0;
    joint_states.name[21] ="laj6";
    joint_states.position[21] = 0.0;
    joint_states.name[22] ="rlaj1";
    joint_states.position[22] = 0.0;
    joint_states.name[23] ="rlaj2";
    joint_states.position[23] = 0.0;
    joint_states.name[24] ="rlaj3";
    joint_states.position[24] = 0.0;
    joint_states.name[25] ="rlaj4";
    joint_states.position[25] = 0.0;
    joint_states.name[26] ="rlaj5";
    joint_states.position[26] = 0.0;
    joint_states.name[27] ="rlaj6";
    joint_states.position[27] = 0.0;
    joint_states.name[28] ="llaj1";
    joint_states.position[28] = 0.0;
    joint_states.name[29] ="llaj2";
    joint_states.position[29] = 0.0;
    joint_states.name[30] ="llaj3";
    joint_states.position[30] = 0.0;
    joint_states.name[31] ="llaj4";
    joint_states.position[31] = 0.0;
    joint_states.name[32] ="llaj5";
    joint_states.position[32] = 0.0;
    joint_states.name[33] ="llaj6";
    joint_states.position[33] = 0.0;
    joint_states.name[34] ="right_wrist_yaw";
    joint_states.position[34] = 0.0;
    joint_states.name[35] ="tj2";
    joint_states.position[35] = 0.0;
    joint_states.name[36] ="tj4";
    joint_states.position[36] = 0.0;
    joint_states.name[37] ="tj5";
    joint_states.position[37] = 0.0;
    joint_states.name[38] ="tj6";
    joint_states.position[38] = 0.0;
    joint_states.name[39] ="ij3";
    joint_states.position[39] = 0.0;
    joint_states.name[40] ="ij4";
    joint_states.position[40] = 0.0;
    joint_states.name[41] ="ij5";
    joint_states.position[41] = 0.0;
    joint_states.name[42] ="mj3";
    joint_states.position[42] = 0.0;
    joint_states.name[43] ="mj4";
    joint_states.position[43] = 0.0;
    joint_states.name[44] ="mj5";
    joint_states.position[44] = 0.0;
    joint_states.name[45] ="rij3";
    joint_states.position[45] = 0.0;
    joint_states.name[46] ="rij4";
    joint_states.position[46] = 0.0;
    joint_states.name[47] ="rij5";
    joint_states.position[47] = 0.0;
    joint_states.name[48] ="raj4";
    joint_states.position[48] = 0.0;
    joint_states.name[49] ="raj5";
    joint_states.position[49] = 0.0;
    joint_states.name[50] ="lij3";
    joint_states.position[50] = 0.0;
    joint_states.name[51] ="lij4";
    joint_states.position[51] = 0.0;
    joint_states.name[52] ="lij5";
    joint_states.position[52] = 0.0;
    joint_states.name[53] ="left_wrist_yaw";
    joint_states.position[53] = 0.0;
    joint_states.name[54] ="ltj2";
    joint_states.position[54] = 0.0;
    joint_states.name[55] ="ltj4";
    joint_states.position[55] = 0.0;
    joint_states.name[56] ="ltj5";
    joint_states.position[56] = 0.0;
    joint_states.name[57] ="ltj6";
    joint_states.position[57] = 0.0;
    joint_states.name[58] ="laij3";
    joint_states.position[58] = 0.0;
    joint_states.name[59] ="laij4";
    joint_states.position[59] = 0.0;
    joint_states.name[60] ="laij5";
    joint_states.position[60] = 0.0;
    joint_states.name[61] ="lmj3";
    joint_states.position[61] = 0.0;
    joint_states.name[62] ="lmj4";
    joint_states.position[62] = 0.0;
    joint_states.name[63] ="lmj5";
    joint_states.position[63] = 0.0;
    joint_states.name[64] ="lrij3";
    joint_states.position[64] = 0.0;
    joint_states.name[65] ="lrij4";
    joint_states.position[65] = 0.0;
    joint_states.name[66] ="lrij5";
    joint_states.position[66] = 0.0;
    joint_states.name[67] ="llij3";
    joint_states.position[67] = 0.0;
    joint_states.name[68] ="llij4";
    joint_states.position[68] = 0.0;
    joint_states.name[69] ="llij5";
    joint_states.position[69] = 0.0;

    joint_pub.write(joint_states);

    while (1) {

      encsTorso->getEncoders(encodersTorso.data());
      encsRArm->getEncoders(encodersRArm.data());
      encsLArm->getEncoders(encodersLArm.data());
      encsHead->getEncoders(encodersHead.data());


      clock_gettime(CLOCK_REALTIME, &currentTime);

      joint_states.header.stamp.sec = currentTime.tv_sec;
      joint_states.header.stamp.nsec = currentTime.tv_nsec;


      // update torso positions
      joint_states.position[0] = encodersTorso[2] * degtorad;
      joint_states.position[1] = encodersTorso[1] * degtorad;
      joint_states.position[2] = encodersTorso[0] * degtorad;

      // update head positions
      joint_states.position[3] = encodersHead[0] * degtorad;
      joint_states.position[4] = encodersHead[1] * degtorad;
      joint_states.position[5] = encodersHead[2] * degtorad;

      // update right arm positions
      joint_states.position[10] = encodersRArm[0] * degtorad;
      joint_states.position[11] = encodersRArm[1] * degtorad;
      joint_states.position[12] = encodersRArm[2] * degtorad;
      joint_states.position[13] = encodersRArm[3] * degtorad;
      joint_states.position[14] = encodersRArm[4] * degtorad;
      joint_states.position[15] = encodersRArm[5] * degtorad;

      // update left arm positions
      joint_states.position[16] = encodersLArm[0] * degtorad;
      joint_states.position[17] = encodersLArm[1] * degtorad;
      joint_states.position[18] = encodersLArm[2] * degtorad;
      joint_states.position[19] = encodersLArm[3] * degtorad;
      joint_states.position[20] = encodersLArm[4] * degtorad;
      joint_states.position[21] = encodersLArm[5] * degtorad;

      joint_pub.write(joint_states);

      Time::delay(0.01);
    }


  return 0;
}
