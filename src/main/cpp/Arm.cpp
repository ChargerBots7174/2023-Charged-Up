#include "Arm.h"

Arm::Arm() {}
Arm::~Arm() {}

float zTarget;
float xTarget;

void Arm::moveArm(double x, double y)
{
    Arm::updateAllEncoders();
    if (y != 1000)
    {
        oldJoint2 = joint2;
    }
    if (y != 6000)
    {
        oldJoint1 = joint1;
    }
    armDiagonal = sqrt((x * x) + (y * y));
    baseAngle = atan(y / x);
    baseInnerAngle = (pi / 2) - baseAngle;
    double innerDiag = sqrt((pow(joint1Length, 2) + pow(armDiagonal, 2) - (2 * (joint1Length) * (armDiagonal) * (cos(baseInnerAngle)))));
    joint2 = acos(((innerDiag * innerDiag) - (joint2Length * joint2Length) - (intakeArmLength * intakeArmLength)) / (-2 * joint2Length * intakeArmLength));

    double joint1part1 = pi - (baseInnerAngle + asin(joint1Length * (sin(baseInnerAngle) / innerDiag)));
    double joint1part2 = pi - (joint2 + asin(joint2Length * (sin(joint2) / innerDiag)));

    double joint1 = joint1part1 + joint1part2;

    joint2 = joint2 * (180 / pi);
    joint1 = joint1 * (180 / pi);

    if (x == 6000)
    {
        joint1 = 135;
        joint2 = 180;
    }
    if (x == 1000)
    {
        joint1 = 180;
        if (currentJoint1Angle > 150)
        {
            joint2 = 0;
        }
        else
        {
            joint2 = oldJoint2;
        }
    }

    joint1Motor.Set(ControlMode::PercentOutput, -(std::clamp(joint1PID.Calculate(currentJoint1Angle, joint1), -0.80, 0.80)));
    joint2Motor.Set(ControlMode::PercentOutput, (std::clamp(joint2PID.Calculate(currentJoint2Angle, joint2), -0.80, 0.80)));

    frc::SmartDashboard::PutNumber("currentJoint1Angle", currentJoint1Angle);
    frc::SmartDashboard::PutNumber("currentJoint2Angle", currentJoint2Angle);
    frc::SmartDashboard::PutNumber("SetJoint1", joint1);
    frc::SmartDashboard::PutNumber("SetJoint2", joint2);
    frc::SmartDashboard::PutNumber("ArmX", x);
    frc::SmartDashboard::PutNumber("ArmY", y);
}

void Arm::resetAllEncoders()
{
    joint2Motor.SetSelectedSensorPosition(0);
    joint1Motor.SetSelectedSensorPosition(0);
}

void Arm::updateAllEncoders()
{
    currentJoint2Angle = (joint2Motor.GetSelectedSensorPosition() / UPPER_ENCODER_TO_ANGLE) + 0;
    currentJoint1Angle = -(joint1Motor.GetSelectedSensorPosition() / LOWER_ENCODER_TO_ANGLE) + 180;
}
