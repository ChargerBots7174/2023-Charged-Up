#pragma once
#include <photonlib/PhotonCamera.h>

#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/Joystick.h>
#include "SwerveDrive.hpp"
#include "AHRS.h"
#include <frc/Encoder.h>
#include "Arm.h"

#include <math.h>
#include <units/angle.h>
#include <units/length.h>

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>
#include <frc/controller/PIDController.h>
#include <frc/PneumaticHub.h>
#include <frc/Compressor.h>
#include <frc/AddressableLED.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SendableChooser.h>

using namespace frc;

class Robot : public frc::TimedRobot
{
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void resetSensors();
    int getLime();

    static constexpr int kLEDs = 129;
    bool idleLED = false;

    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    frc::AddressableLED m_led{9};
    std::array<frc::AddressableLED::LEDData, kLEDs>
        m_ledBuffer; // Reuse the buffer
    // Store what the last hue of the first pixel is
    int firstPixelHue = 0;

    frc::SendableChooser<std::string> m_chooser;
    const std::string kDefaultTest = "Comp mode";
    const std::string ktestMode = "Test Mode";

    frc::SendableChooser<std::string> m_steerPID;
    const std::string kPIDON = "PID ON";
    const std::string kPIDOFF = "PID OFF";

    frc::SendableChooser<std::string> m_robothead;
    const std::string kStaright = "Robot Facing Stright";
    const std::string kBack = "Robot Facing Back";

    frc::SendableChooser<std::string> m_auton;
    const std::string kMID = "MID position";
    const std::string kSIDE = "SIDE position";

    frc::SendableChooser<std::string> m_cone;
    const std::string kHighCone = "High Cone";
    const std::string kMidCone = "Mid Cone";

    frc::XboxController xboxController{0};
    frc::XboxController xboxController2{1};


    double limeOffset;

    double frontRightDriveEncoder = 0;
    double frontLeftDriveEncoder = 0;
    double backRightDriveEncoder = 0;
    double backLeftDriveEncoder = 0;
    double currDriveEnc = 0;
    Arm testArm;

    double armX = 0;
    double armY = 0;

    double frontRightAngle = 0;
    double frontLeftAngle = 0;
    double backRightAngle = 0;
    double backLeftAngle = 0;

    double tx = 0;
    double targetOffsetAngle_Vertical = 0;

    double encDistance = (6.54 * 2048) / (4 * 3.1415269);

    WPI_TalonSRX frontLeftDriveMotor = 20;
    WPI_TalonSRX frontLeftAngleMotor = 21;

    WPI_TalonSRX frontRightDriveMotor = 10;
    WPI_TalonSRX frontRightAngleMotor = 11;

    WPI_TalonSRX backLeftDriveMotor = 30;
    WPI_TalonSRX backLeftAngleMotor = 31;

    WPI_TalonSRX backRightDriveMotor = 40;
    WPI_TalonSRX backRightAngleMotor = 41;

    WPI_TalonSRX lowerArmMotor = 50;
    WPI_TalonSRX upperArmMotor = 51;

private:
    // CONTROLLER VALUES
    typedef struct inputs{
        float x;
        float y;
        float z;
    }inputs;

    inputs current;
    inputs filtered;
    double filterValue = 0.2;


    double xOffSet;
    bool targetFound = false;
    double finalShotVel = 0;
    double finalShotSpeed = 0;
    Timer m_timer;

    double limitX = 0;
    double limitY = 0;

    double pitch = 0;
    double currentHead = 0;

    double currTime = 0;
    bool reachedStation = false;

    frc2::PIDController limelight{0.0225, 0, 0};    // possibly change the pid to 0.025
    frc2::PIDController stationPID{0.011, 0, 0};     // possibly change the pid 0.05
    frc2::PIDController yawPID{0.0075, 0, 0};         // possibly change the pid 0.05

    double speedMul = 0;
    AHRS *ahrs;
    double nav_yaw = 0;

    int currentState = 0;

    double driveX, driveY, driveZ = 0;
    double maxSpeed = 0;

    // PNEUMATICS
    frc::Compressor pcmCompressor{2, frc::PneumaticsModuleType::REVPH};
    bool pressureSwitch = pcmCompressor.GetPressureSwitchValue();
    frc::DoubleSolenoid Grabber{2, frc::PneumaticsModuleType::REVPH, 0, 7};

    SwerveDrive driveTrain;
    // start of photon align
    //  Change this to match the name of your camera
    photonlib::PhotonCamera camera{"HD_USB_Camera"}; // change to name of camera used arduino camera
    // PID constants should be tuned per robot
    frc2::PIDController photoncontroller{.1, 0, 0}; // tune the PID numbers
    // end of photon align

    void filter(double filterAmount, inputs *current, inputs *previous);

    void updateAllEncoders()
    {
        frontRightAngle = frontRightAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;
        frontLeftAngle = frontLeftAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;
        backRightAngle = backRightAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;
        backLeftAngle = backLeftAngleMotor.GetSelectedSensorPosition() * ENCODER_TO_ANGLE;

        frontRightDriveEncoder = frontRightDriveMotor.GetSelectedSensorPosition();
        frontLeftDriveEncoder = frontLeftDriveMotor.GetSelectedSensorPosition();
        backRightDriveEncoder = backRightDriveMotor.GetSelectedSensorPosition();
        backLeftDriveEncoder = backLeftDriveMotor.GetSelectedSensorPosition();
    }

    void resetAllEncoders()
    {
        frontRightAngleMotor.SetSelectedSensorPosition(0);
        frontLeftAngleMotor.SetSelectedSensorPosition(0);
        backRightAngleMotor.SetSelectedSensorPosition(0);
        backLeftAngleMotor.SetSelectedSensorPosition(0);

        frontRightDriveMotor.SetSelectedSensorPosition(0);
        frontLeftDriveMotor.SetSelectedSensorPosition(0);
        backRightDriveMotor.SetSelectedSensorPosition(0);
        backLeftDriveMotor.SetSelectedSensorPosition(0);
    }
};