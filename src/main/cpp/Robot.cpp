#include "Robot.h"
#include "SwerveDrive.hpp"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>
#include "frc/DigitalInput.h"

#include <frc/TimedRobot.h>
#include <frc/Timer.h>

#include <cameraserver/CameraServer.h>

#include <photonlib/PhotonUtils.h>

#define _USE_MATH_DEFINES

void Robot::RobotInit()
{
    ahrs = new AHRS(SPI::Port::kMXP);
    resetSensors();
    frc::CameraServer::StartAutomaticCapture();
}

void Robot::AutonomousInit()
{
    resetSensors();
    resetAllEncoders();
    m_timer.Reset();
    m_timer.Start();
    targetFound = false;
}

void Robot::TeleopInit()
{

    resetSensors();
    resetAllEncoders();
}


void Robot::TeleopPeriodic()
{
     driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1);

    frc::SmartDashboard::PutNumber("back left vel", backLeftDriveMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("front left vel", frontLeftDriveMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("back right vel", backRightDriveMotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("front right vel", frontRightDriveMotor.GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("back left pos", backLeftDriveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("front left pos", frontLeftDriveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("back right pos", backRightDriveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("front right pos", frontRightDriveMotor.GetSelectedSensorPosition());

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 26.2; // 24.65 //26.75

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 18.0;

    // distance from the target to the floor
    double goalHeightInches = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians);

    // DRIVE
    nav_yaw = -ahrs->GetYaw();
  
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("photonvision")->PutNumber("camMode", 1);


    if (joystickController.GetRawButton(3) > 0.5 || buttonBoard.GetRawButton(9) > 0.5 || buttonBoard.GetRawButton(10) > 0.5)
    { // changed from 4 to 3
        driveX = 0;
        driveY = 0;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
        tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) - 2;
        targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0) - 2;
        limelight.EnableContinuousInput(-300, 300);
        driveZ = -limelight.Calculate(tx, 0);
    }
    else if (joystickController.GetRawButton(2) == 0)
    {
        driveX = joystickController.GetX() * 0.25;
        driveY = joystickController.GetY() * 0.25;
        if (joystickController.GetRawButton(7) > 0.5)
        { // changed 8 to 7
            driveZ = 0;
        }
        else
        {
            driveZ = joystickController.GetTwist() * 0.25;
        }
        maxSpeed = 1;
    }
    else if (joystickController.GetTrigger() > 0.5)
    {
        driveX = joystickController.GetX() * 0.35;
        driveY = joystickController.GetY() * 0.35;
        if (joystickController.GetRawButton(7) > 0.5)
        { // changed 8 to 7
            driveZ = 0;
        }
        else
        {
            driveZ = joystickController.GetTwist() * 0.35;
        }
        maxSpeed = 1;
    }
    else
    {
        driveX = joystickController.GetX();
        driveY = joystickController.GetY();
        if (joystickController.GetRawButton(7) > 0.5)
        { // changed 8 to 7
            driveZ = 0;
        }
        else
        {
            driveZ = joystickController.GetTwist();
        }
        maxSpeed = 1;
    }
    // END

    

//start of the photonvision 
if (joystickController.GetRawButton(3) > 0.5) {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();

    if (result.HasTargets()) {
      // Rotation speed is the output of the PID controller
      driveZ = -photoncontroller.Calculate(result.GetBestTarget().GetYaw(), 0);
    } 
    
    else {
      // If we have no targets, stay still.
      driveZ = 0;
    }
    }
   else {
    // Manual Driver Mode
    driveZ = joystickController.GetTwist() > 0.5;
  }

  // Use our forward/turn speeds to control the drivetrain
  driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1);
};


void Robot::AutonomousPeriodic()
{
   

    

    nav_yaw = -ahrs->GetYaw();

    double targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0) - 2;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 26.2; // 24.65

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 18.0;

    // distance from the target to the floor
    double goalHeightInches = 104.0;


    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / tan(angleToGoalRadians);
    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) - 2;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
    frc::SmartDashboard::PutNumber("CitrusLumen", tx);

    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1);
}

void Robot::resetSensors()
{
    ahrs->Reset();
    ahrs->ZeroYaw();
    driveTrain.resetAllEncoders();

    frontRightAngleMotor.SetSelectedSensorPosition(0);
    frontLeftAngleMotor.SetSelectedSensorPosition(0);
    backRightAngleMotor.SetSelectedSensorPosition(0);
    backLeftAngleMotor.SetSelectedSensorPosition(0);

    frontRightDriveMotor.SetSelectedSensorPosition(0);
    frontLeftDriveMotor.SetSelectedSensorPosition(0);
    backRightDriveMotor.SetSelectedSensorPosition(0);
    backLeftDriveMotor.SetSelectedSensorPosition(0);

    frontLeftAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontLeftAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontLeftAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontLeftAngleMotor.EnableCurrentLimit(true);

    frontRightAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontRightAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontRightAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontRightAngleMotor.EnableCurrentLimit(true);

    backLeftAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backLeftAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backLeftAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backLeftAngleMotor.EnableCurrentLimit(true);

    backRightAngleMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backRightAngleMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backRightAngleMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backRightAngleMotor.EnableCurrentLimit(true);

    frontLeftDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontLeftDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontLeftDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontLeftDriveMotor.EnableCurrentLimit(true);

    frontRightDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    frontRightDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    frontRightDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    frontRightDriveMotor.EnableCurrentLimit(true);

    backLeftDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backLeftDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backLeftDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backLeftDriveMotor.EnableCurrentLimit(true);

    backRightDriveMotor.ConfigPeakCurrentLimit(30);       // don't activate current limit until current exceeds 30 A ...
    backRightDriveMotor.ConfigPeakCurrentDuration(100);   // ... for at least 100 ms
    backRightDriveMotor.ConfigContinuousCurrentLimit(20); // once current-limiting is actived, hold at 20A
    backRightDriveMotor.EnableCurrentLimit(true);
}

void Robot::RobotPeriodic() {}
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif