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
#define FILTER 0.8



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
    filtered.x = 0;
    filtered.z = 0;
    filtered.y = 0;
    limelight.EnableContinuousInput(-27,27);
    frc::SmartDashboard::PutNumber("random test z", filterValue);
    frc::SmartDashboard::PutNumber("Test Angle", 130);

    
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
    yawPID.EnableContinuousInput(-180, 180);

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
        driveX = xboxController.GetLeftX() * 0.25;
        driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 180), -0.5, 0.5);
    }
    else if (xboxController.GetRightBumper()){
        driveY = xboxController.GetLeftY() * 0.25;
        driveX = xboxController.GetLeftX() * 0.25;
        driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 0), -0.5, 0.5);
    }
    else if (xboxController.GetXButton()){
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
        tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
        frc::SmartDashboard::PutNumber("limelight offset", tx);

        driveY = xboxController.GetLeftY() * 0.25;
        driveX = std::clamp(limelight.Calculate(tx, 0), -0.50, 0.50);
        driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 0), -0.5, 0.5);
    }
    else
    {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
        driveX = xboxController.GetLeftX() * 0.35;
        driveY = xboxController.GetLeftY() * 0.35;
        driveZ = xboxController.GetRightX() * 0.35;
        maxSpeed = 1;
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
        armX = armX + (xboxController2.GetLeftX() * 0.3);
        armY = armY - (xboxController2.GetLeftY() * 0.3);
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
    
    // driveX = floor(driveX * 10.) / 10.;
    // driveY = floor(driveY * 10.) / 10.;
    
    frc::SmartDashboard::PutNumber("filter X", driveX);
    frc::SmartDashboard::PutNumber("filter Y", driveY);
    frc::SmartDashboard::PutNumber("filter Z", driveZ);

    frc::SmartDashboard::PutNumber("joystick X", current.x);
    frc::SmartDashboard::PutNumber("joystick y", current.y);
    frc::SmartDashboard::PutNumber("joystick z", current.z);

    
    filterValue = frc::SmartDashboard::GetNumber("random test z", 0.1);

    //filter inputs
    current.x = driveX;
    current.y = driveY;
    current.z = driveZ;
    filter(filterValue, &current, &filtered);
    driveX = filtered.x;
    driveY = filtered.y;
    driveZ = filtered.z;

    
    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1, m_robothead.GetSelected());
    testArm.moveArm(armX, armY);
}

void Robot::AutonomousPeriodic()
{
    
    pcmCompressor.EnableDigital();
    nav_yaw = -ahrs->GetYaw();
    pitch = ahrs->GetRoll();

    yawPID.EnableContinuousInput(-180, 180);
    pcmCompressor.EnableDigital();

    currTime = double(m_timer.Get());
    if (currTime < 1.5)
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
    else if (currTime < 2.5)
    {
        driveY = 0.3;
    }
    else if (currTime < 3)
    {
        Grabber.Set(frc::DoubleSolenoid::Value::kForward);
        driveY = 0;
    }
    else if (currTime < 6.75)
    {
        driveY = -0.4;
        armX = 1000;
        armY = 1000;
    }
    else if (currTime < 7.25)
    {
        driveY = 0;
    }
    else
    {
        if (m_auton.GetSelected() == kMID)
        {
            if (reachedStation == false)
            {
                driveY = 0.6; //remove - is we cross community
                if (pitch > 15 || -15 > pitch)
                {
                    driveY = 0;
                    reachedStation = true;
                }
            }
            else
            {
                driveY = std::clamp(stationPID.Calculate(pitch, 0), -0.35, 0.35);
            }
            driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 0), -1.0, 1.0);
        }
        else
        {
                driveY = 0;
                driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 180), -0.5, 0.5);
        }
    }
    if (currTime < 7.25){
        driveZ = -std::clamp(yawPID.Calculate(nav_yaw, 0), -1.0, 1.0);
    }

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

    driveTrain.drive(driveX, driveY, driveZ, nav_yaw, 1, m_robothead.GetSelected());
    testArm.moveArm(armX, armY);
}

void Robot::filter(double filter, inputs *current, inputs *previous){
    //(1-a)*filtered + a*current
    double ts = 1.0/50.0; //time step
    double a = ts/(filter+ts);
    previous->x = (1.0 - a)*previous->x + a*current->x;
    previous->y = (1.0 - a)*previous->y + a*current->y;
    previous->z = (1.0 - a)*previous->z + a*current->z;

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