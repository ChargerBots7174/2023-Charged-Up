#include "SwerveDrive.hpp"

SwerveDrive::SwerveDrive()
{
}
SwerveDrive::~SwerveDrive() {}

void SwerveDrive::drive(double xv, double yv, double omega, double yaw, double maxSpeed, std::string robotHead)
{
    if (robotHead == "Robot Facing Back")
    {
        xv = -xv;
        yv = -yv;
    }

    updateAllEncoders();
    double r = sqrt((robotLength * robotLength) + (robotWidth * robotWidth));
    yv *= -1;

    if (yv < YvdriftOffset && yv > -YvdriftOffset)
    {
        yv = 0;
    }
    if (xv < XvdriftOffset && xv > -XvdriftOffset)
    {
        xv = 0;
    }
    if (omega < twistdriftOffset && omega > -twistdriftOffset)
    {
        omega = 0;
        gyroPID.EnableContinuousInput(-180, 180);
        gyroMul = gyroPID.Calculate(yaw, 0);
    }
    robotHeading = (-yaw * 2 * acos(0)) / 180;

    double temp = yv * cos(robotHeading) + xv * sin(robotHeading);
    xv = -yv * sin(robotHeading) + xv * cos(robotHeading);
    yv = temp;

    double a = xv - omega * (robotLength / r);
    double b = xv + omega * (robotLength / r);
    double c = yv - omega * (robotWidth / r);
    double d = yv + omega * (robotWidth / r);

    

    // MOTOR SPEED CALCULATIONS

    backRightSpeed = sqrt(((a * a) + (c * c)));
    backLeftSpeed = sqrt(((a * a)) + ((d * d)));
    frontRightSpeed = sqrt((b * b)) + ((c * c));
    frontLeftSpeed = sqrt((b * b)) + ((d * d));

    // MOTOR ANGLE CALCULATIONS
    backLeftAngleCalc = 180 * atan2(a, d) / (2 * acos(0));
    backRightAngleCalc = 180 * atan2(a, c) / (2 * acos(0));
    frontLeftAngleCalc = 180 * atan2(b, d) / (2 * acos(0));
    frontRightAngleCalc = 180 * atan2(b, c) / (2 * acos(0));

    // SET DRIVE MOTORS
    frontLeftDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * maxSpeed));
    frontRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * maxSpeed));
    backRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * maxSpeed));
    backLeftDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed * maxSpeed));

    steerPID.EnableContinuousInput(-180, 180);
    drivePID.EnableContinuousInput(-1, 1);
    // SET ANGLE MOTORS
    frontLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontLeftAngle, frontLeftAngleCalc)));
    frontRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontRightAngle, frontRightAngleCalc)));
    backRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backRightAngle, backRightAngleCalc)));
    backLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backLeftAngle, backLeftAngleCalc)));
}

double SwerveDrive::map(double val, double x, double y, double j, double k)
{
    return j + ((val - x) * (k - j)) / (y - x);
}

void SwerveDrive::driveAutonomous(double xv, double yv, double omega, double yaw, double maxSpeed)
{
    updateAllEncoders();
    double r = sqrt((robotLength * robotLength) + (robotWidth * robotWidth));
    yv *= -1;

    robotHeading = (-yaw * 2 * acos(0)) / 180;

    double temp = yv * cos(robotHeading) + xv * sin(robotHeading);
    xv = -yv * sin(robotHeading) + xv * cos(robotHeading);
    yv = temp;

    double a = xv - omega * (robotLength / r);
    double b = xv + omega * (robotLength / r);
    double c = yv - omega * (robotWidth / r);
    double d = yv + omega * (robotWidth / r);

    // MOTOR SPEED CALCULATIONS
    backRightSpeed = sqrt(((a * a) + (d * d)));
    backLeftSpeed = sqrt(((a * a)) + ((c * c)));
    frontRightSpeed = sqrt(((b * b)) + ((d * d)));
    frontLeftSpeed = sqrt(((b * b)) + ((c * c)));

    // MOTOR ANGLE CALCULATIONS
    backLeftAngleCalc = 180 * atan2(a, d) / (2 * acos(0));
    backRightAngleCalc = 180 * atan2(a, c) / (2 * acos(0));
    frontLeftAngleCalc = 180 * atan2(b, d) / (2 * acos(0));
    frontRightAngleCalc = 180 * atan2(b, c) / (2 * acos(0));

    // SET DRIVE MOTORS
    frontLeftDriveMotor.Set(ControlMode::PercentOutput, (frontLeftSpeed));
    frontRightDriveMotor.Set(ControlMode::PercentOutput, (frontRightSpeed));
    backRightDriveMotor.Set(ControlMode::PercentOutput, (backRightSpeed));
    backLeftDriveMotor.Set(ControlMode::PercentOutput, (backLeftSpeed));

    steerPID.EnableContinuousInput(-180, 180);
    drivePID.EnableContinuousInput(-1, 1);

    // SET ANGLE MOTORS
    frontLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontLeftAngle, frontLeftAngleCalc)));
    frontRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(frontRightAngle, frontRightAngleCalc)));
    backRightAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backRightAngle, backRightAngleCalc)));
    backLeftAngleMotor.Set(ControlMode::PercentOutput, (steerPID.Calculate(backLeftAngle, backLeftAngleCalc)));
}

void SwerveDrive::updateAllEncoders()
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

void SwerveDrive::resetAllEncoders()
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