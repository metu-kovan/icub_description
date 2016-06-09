// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "../include/sensor_msgs_JointState.h"

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;


int main(int argc, char const *argv[]) {
    Network yarp;

    Node node("/icubSim/state_publisher");

    Property options_torso;
    options_torso.put("device", "remote_controlboard");
    options_torso.put("local", "/torso/client");   //local port names
    options_torso.put("remote", "/icubSim/torso");
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
    options_right_arm.put("remote", "/icubSim/right_arm");
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
    options_left_arm.put("remote", "/icubSim/left_arm");
    // create a device
    PolyDriver lArmDevice(options_left_arm);
    if (!lArmDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *posTorso;
    IEncoders *encsTorso;
    IPositionControl *posRArm;
    IEncoders *encsRArm;
    IPositionControl *posLArm;
    IEncoders *encsLArm;

    torsoDevice.view(posTorso);
    torsoDevice.view(encsTorso);
    rArmDevice.view(posRArm);
    rArmDevice.view(encsRArm);
    lArmDevice.view(posLArm);
    lArmDevice.view(encsLArm);


    int nj=0;
    posTorso->getAxes(&nj);
    Vector encodersTorso;
    encodersTorso.resize(nj);

    nj=0;
    posRArm->getAxes(&nj);
    Vector encodersRArm;
    encodersRArm.resize(nj);

    nj=0;
    posLArm->getAxes(&nj);
    Vector encodersLArm;
    encodersLArm.resize(nj);



    // while(!encsTorso->getEncoders(encodersTorso.data()))
    // {
    //     Time::delay(0.1);
    //     printf(".");
    // }

    yarp::os::Publisher<sensor_msgs_JointState> joint_pub;
    if (!joint_pub.topic("/joint_states")) {
        std::cerr<< "Failed to create publisher to /chatter\n";
        return -1;
    }

    float degtorad = 0.0174532925;

    while (1) {

      encsTorso->getEncoders(encodersTorso.data());
      encsRArm->getEncoders(encodersRArm.data());
      encsLArm->getEncoders(encodersLArm.data());


      sensor_msgs_JointState joint_states;

      joint_states.header.stamp.sec = Time::now();
      joint_states.name.resize(68);
      joint_states.position.resize(68);
      joint_states.name[0] ="j1";
      joint_states.position[0] = encodersTorso[2] * degtorad;
      joint_states.name[1] ="j2";
      joint_states.position[1] = encodersTorso[1] * degtorad;
      joint_states.name[2] ="j3";
      joint_states.position[2] = encodersTorso[0] * degtorad;
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
      joint_states.position[10] = encodersRArm[0] * degtorad;
      joint_states.name[11] ="raj2";
      joint_states.position[11] = encodersRArm[1] * degtorad;
      joint_states.name[12] ="raj3";
      joint_states.position[12] = encodersRArm[2] * degtorad;
      joint_states.name[13] ="raj4";
      joint_states.position[13] = encodersRArm[3] * degtorad;
      joint_states.name[14] ="raj5";
      joint_states.position[14] = encodersRArm[4] * degtorad;
      joint_states.name[15] ="raj6";
      joint_states.position[15] = encodersRArm[5] * degtorad;
      joint_states.name[16] ="laj1";
      joint_states.position[16] = encodersLArm[0] * degtorad;
      joint_states.name[17] ="laj2";
      joint_states.position[17] = encodersLArm[1] * degtorad;
      joint_states.name[18] ="laj3";
      joint_states.position[18] = encodersLArm[2] * degtorad;
      joint_states.name[19] ="laj4";
      joint_states.position[19] = encodersLArm[3] * degtorad;
      joint_states.name[20] ="laj5";
      joint_states.position[20] = encodersLArm[4] * degtorad;
      joint_states.name[21] ="laj6";
      joint_states.position[21] = encodersLArm[5] * degtorad;
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
      joint_states.name[47] ="raj4";
      joint_states.position[47] = 0.0;
      joint_states.name[48] ="raj5";
      joint_states.position[48] = 0.0;
      joint_states.name[49] ="lij3";
      joint_states.position[49] = 0.0;
      joint_states.name[50] ="lij4";
      joint_states.position[50] = 0.0;
      joint_states.name[51] ="lij5";
      joint_states.position[51] = 0.0;
      joint_states.name[52] ="left_wrist_yaw";
      joint_states.position[52] = 0.0;
      joint_states.name[53] ="ltj2";
      joint_states.position[53] = 0.0;
      joint_states.name[54] ="ltj4";
      joint_states.position[54] = 0.0;
      joint_states.name[55] ="ltj5";
      joint_states.position[55] = 0.0;
      joint_states.name[56] ="ltj6";
      joint_states.position[56] = 0.0;
      joint_states.name[57] ="laij3";
      joint_states.position[57] = 0.0;
      joint_states.name[58] ="laij4";
      joint_states.position[58] = 0.0;
      joint_states.name[59] ="laij5";
      joint_states.position[59] = 0.0;
      joint_states.name[60] ="lmj3";
      joint_states.position[60] = 0.0;
      joint_states.name[61] ="lmj4";
      joint_states.position[61] = 0.0;
      joint_states.name[62] ="lrij3";
      joint_states.position[62] = 0.0;
      joint_states.name[63] ="lrij4";
      joint_states.position[63] = 0.0;
      joint_states.name[64] ="lrij5";
      joint_states.position[64] = 0.0;
      joint_states.name[65] ="llij3";
      joint_states.position[65] = 0.0;
      joint_states.name[66] ="llij4";
      joint_states.position[66] = 0.0;
      joint_states.name[67] ="llij5";
      joint_states.position[67] = 0.0;


      joint_pub.write(joint_states);



      Time::delay(0.2);
    }

    // joint_pub.close();

  return 0;
}
