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
#include "Arm.h"

#include <frc/TimedRobot.h>
#include <frc/Timer.h>

#include <cameraserver/CameraServer.h>

#include <photonlib/PhotonUtils.h>

#define _USE_MATH_DEFINES

#define BIAS 0.4

void Robot::RobotInit()
{
    testArm.resetAllEncoders();
    ahrs = new AHRS(SPI::Port::kMXP);
    resetSensors();
    frc::CameraServer::StartAutomaticCapture();

    m_chooser.SetDefaultOption(kDefaultTest, kDefaultTest);
    m_chooser.AddOption(ktestMode, ktestMode);
    frc::SmartDashboard::PutData("Test Mode", &m_chooser);

    m_robothead.SetDefaultOption(kStaright, kStaright);
    m_robothead.AddOption(kBack, kBack);
    frc::SmartDashboard::PutData("NavX heading", &m_robothead);

    m_steerPID.SetDefaultOption(kPIDOFF, kPIDOFF);
    m_steerPID.AddOption(kPIDON, kPIDON);
    frc::SmartDashboard::PutData("Steer PID", &m_steerPID);

    m_auton.SetDefaultOption(kMID, kMID);
    m_auton.AddOption(kSIDE, kSIDE);
    frc::SmartDashboard::PutData("Auton", &m_auton);

    m_cone.SetDefaultOption(kMidCone, kMidCone);
    m_cone.AddOption(kHighCone, kHighCone);
    frc::SmartDashboard::PutData("Cone Drop", &m_cone);

    m_led.SetLength(kLEDs);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void Robot::AutonomousInit()
{
    resetSensors();
    resetAllEncoders();
    m_timer.Reset();
    m_timer.Start();
    targetFound = false;
    m_led.SetLength(kLEDs);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void Robot::TeleopInit()
{
    if (m_chooser.GetSelected() == ktestMode)
    {
        resetSensors();
        resetAllEncoders();
        testArm.resetAllEncoders();
    }
    Grabber.Set(frc::DoubleSolenoid::Value::kReverse);
    m_led.SetLength(kLEDs);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void Robot::TeleopPeriodic()
{
    pcmCompressor.EnableDigital();

    // open and close claw

    if (xboxController2.GetLeftBumper())
    {
        Grabber.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if (xboxController2.GetRightBumper())
    {
        Grabber.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    // DRIVE
    nav_yaw = -ahrs->GetYaw();

    if (xboxController.GetRightTriggerAxis() == 1)
    {
        driveX = xboxController.GetLeftX();
        driveY = xboxController.GetLeftY();
        driveZ = xboxController.GetRightX();
        maxSpeed = 1;
    }
    else if (xboxController.GetLeftTriggerAxis() == 1)
    {
        driveX = xboxController.GetLeftX() * 0.15;
        driveY = xboxController.GetLeftY() * 0.15;
        driveZ = xboxController.GetRightX() * 0.15;
        maxSpeed = 1;
    }
    else if (xboxController.GetLeftBumper()){
        driveY = xboxController.GetLeftY() * 0.25;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
        tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
        frc::SmartDashboard::PutNumber("Limelight X Offset", tx);
        driveX = std::clamp(limelight.Calculate(tx, 0), -0.50, 0.50);
        driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 0), -0.75, 0.75);
        frc::SmartDashboard::PutNumber("Limelight Swerve Speed", driveX);
    }
    else
    {
        driveX = xboxController.GetLeftX() * 0.5;
        driveY = xboxController.GetLeftY() * 0.5;
        driveZ = xboxController.GetRightX() * 0.5;
        maxSpeed = 1;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
    }
    if (xboxController.GetRightX() < 0.08 && xboxController.GetRightX() > -0.08 && m_steerPID.GetSelected() == kPIDON)
    {
        driveZ = -std::clamp(yawPID.Calculate(nav_yaw, currentHead), -1.0, 1.0);
    }
    if (xboxController.GetRightX() > 0.08 || xboxController.GetRightX() < -0.08)
    {
        currentHead = nav_yaw;
    }

    // ARM
    if (xboxController2.GetLeftX() > 0.1 || xboxController2.GetLeftX() < -0.1 || xboxController2.GetLeftY() > 0.1 || xboxController2.GetLeftY() < -0.1)
    {
        armX = armX + (xboxController2.GetLeftX() * 0.4);
        armY = armY - (xboxController2.GetLeftY() * 0.4);
    }
    else if (xboxController2.GetYButton())
    { // high
        armX = 6000;
        armY = 6000;
    }
    else if (xboxController2.GetAButton())
    { // ground
        armX = 25;
        armY = 4;
    }
    else if (xboxController2.GetXButton())
    { // mid
        armX = 34;
        armY = 49;
    }
    else if (xboxController2.GetBButton() || xboxController.GetLeftStickButton() || xboxController.GetRightStickButton())
    {
        armX = 1000;
        armY = 1000;
    }

    if (xboxController.GetXButton())
    {
        idleLED = false;
        for (int i = 0; i < kLEDs; i++)
        {
            m_ledBuffer[i].SetRGB(0, 0, 255);
        }
    }
    else if (xboxController.GetAButton())
    {
        idleLED = false;
        for (int i = 0; i < kLEDs; i++)
        {
            m_ledBuffer[i].SetRGB(250, 150, 0);
        }
    }
    else if (xboxController.GetBButton() || idleLED == true)
    {
        idleLED = true;
        for (int i = 0; i < kLEDs; i++)
        {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            const auto pixelHue = (firstPixelHue + (i * 180 / kLEDs)) % 180;
            // Set the value
            m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        firstPixelHue += 3;
        // Check bounds
        firstPixelHue %= 180;
    }

    m_led.SetData(m_ledBuffer);
    // Use our forward/turn speeds to control the drivetrain

    frc::SmartDashboard::PutNumber("driveZ", driveZ);
    frc::SmartDashboard::PutNumber("Yaw", nav_yaw);
    frc::SmartDashboard::PutNumber("currHead", currentHead);
    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1, m_robothead.GetSelected());
    testArm.moveArm(armX, armY);
}

void Robot::AutonomousPeriodic()
{
    nav_yaw = -ahrs->GetYaw();
    pitch = ahrs->GetRoll();

    pcmCompressor.EnableDigital();

    currTime = double(m_timer.Get());
    if (currTime < 2)
    {
        Grabber.Set(frc::DoubleSolenoid::Value::kReverse);
        if (m_cone.GetSelected() == kHighCone)
        {
            armX = 6000;
            armY = 6000;
        }
        else
        {
            armX = 38;
            armY = 48;
        }
    }
    else if (currTime < 2.25)
    {
        driveY = 0.081;
    }
    else if (currTime < 4)
    {
        driveY = 0.2;
    }
    else if (currTime < 5)
    {
        Grabber.Set(frc::DoubleSolenoid::Value::kForward);
        driveY = 0;
    }
    else if (currTime < 6)
    {
        driveY = -0.3;
    }
    else if (currTime < 7)
    {
        driveY = 0;
        armX = 1000;
        armY = 1000;
    }
    else if (currTime < 8.5)
    {
        driveY = -0.3;
        currDriveEnc = frontRightDriveMotor.GetSelectedSensorPosition();
    }
    else
    {
        if (m_auton.GetSelected() == kMID)
        {
            if (reachedStation == false)
            {
                driveY = -0.5;
                if (pitch > 15)
                {
                    driveY = 0;
                    reachedStation = true;
                }
            }
            else
            {
                driveY = std::clamp(stationPID.Calculate(pitch, 0), -0.25, 0.25);
            }
        }
        else
        {
            if (currTime < 9.5)
            {
                driveY = -0.3;
            }
            else
            {
                driveY = 0;
            }
        }
    }
    frc::SmartDashboard::PutNumber("ARE YOU ON CHARGING STATION?", reachedStation);
    frc::SmartDashboard::PutNumber("PITCHHHHHHHHHHHHHHHHHHHHHHH", pitch);

    idleLED = true;
    for (int i = 0; i < kLEDs; i++)
    {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        const auto pixelHue = (firstPixelHue + (i * 180 / kLEDs)) % 180;
        // Set the value
        m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    // Check bounds
    firstPixelHue %= 180;

    m_led.SetData(m_ledBuffer);

    idleLED = true;

    driveZ = -std::clamp(yawPID.Calculate(nav_yaw, currentHead), -1.0, 1.0);
    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1, m_robothead.GetSelected());
    testArm.moveArm(armX, armY);
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