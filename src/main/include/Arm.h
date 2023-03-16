#ifndef ARM_H
#define ARM_H
#pragma once

#include <ctre/Phoenix.h>
#include "cmath"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

using namespace frc;

class Arm
{
private:
    WPI_TalonSRX joint1Motor = 50;
    WPI_TalonSRX joint2Motor = 51;

    double pi = 3.14159265358;

    double UPPER_ENCODER_TO_ANGLE = (204800 * (40 / 12)) / 360;
    double LOWER_ENCODER_TO_ANGLE = (204800 * (52 / 12)) / 360;

    double joint1Length = 23.75;
    double joint2Length = 27;
    double intakeArmLength = 38;

    double currentJoint2Angle = 0;
    double currentJoint1Angle = 0;

    double baseAngle = 0;
    double innerDiag = 0;
    double baseInnerAngle = 0;
    double armDiagonal = 0;
    double joint1 = 0;
    double joint2 = 0;
    double oldJoint2 = 0;
    double oldJoint1 = 0;

    frc2::PIDController joint1PID{0.05, 0, 0};
    frc2::PIDController joint2PID{0.05, 0, 0};
    // protobot::ProtoPID myjoint1PID;
    // protobot::ProtoPID myjoint2PID;

public:
    void moveArm(double x, double z);
    void updateAllEncoders();
    void resetAllEncoders();
    Arm();
    ~Arm();
};

#endif