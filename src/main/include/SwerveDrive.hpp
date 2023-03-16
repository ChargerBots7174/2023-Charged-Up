#ifndef SWERVEDRIVE_H
#define SWERVEDRIVE_H
#pragma once

#include <ctre/Phoenix.h>
#include "cmath"
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/controller/PIDController.h>
#include "AHRS.h"

#define GEAR_RATIO 6.54
#define ENCODER_TO_INCHES 6.54*2048/4*3.14159265 ; //2048 number
#define ENCODER_TO_ANGLE 35/3072


using namespace frc;

class SwerveDrive  
{
	private:
		WPI_TalonSRX frontLeftDriveMotor = 20;
		WPI_TalonSRX frontLeftAngleMotor = 21;

		WPI_TalonSRX frontRightDriveMotor = 10;
		WPI_TalonSRX frontRightAngleMotor = 11;

		WPI_TalonSRX backLeftDriveMotor = 30;
		WPI_TalonSRX backLeftAngleMotor = 31;

		WPI_TalonSRX backRightDriveMotor = 40;
		WPI_TalonSRX backRightAngleMotor = 41;
		
		frc2::PIDController steerPID{0.00675, 0, 0};
		frc2::PIDController drivePID{1, 0, 0};
		frc2::PIDController gyroPID{0.45, 0, 0};
		
		double robotWidth = 30;
		double robotLength = 30;
		double gyroMul = 0;

		double frontRightAngle = 0;
		double frontLeftAngle = 0;
    	double backRightAngle = 0;
    	double backLeftAngle = 0;
		
		double frontRightDriveEncoder = 0;
		double frontLeftDriveEncoder = 0;
    	double backRightDriveEncoder = 0;
    	double backLeftDriveEncoder = 0;

		double robotHeading = 0;

		double backRightSpeed = 0;
		double backLeftSpeed = 0;
		double frontRightSpeed = 0;
		double frontLeftSpeed = 0;

		double backRightAngleCalc = 0;
		double backLeftAngleCalc = 0;
		double frontRightAngleCalc = 0;
		double frontLeftAngleCalc = 0;

		
		double XvdriftOffset = 0.08; 
		double YvdriftOffset = 0.08; 
		double twistdriftOffset = 0.2; 
		double negtwistdriftOffset = 0.2; 

		
	public:
		void drive(double xv, double yv, double omega, double yaw, double maxSpeed);
 		void driveAutonomous(double xv, double yv, double omega,double yaw, double maxSpeed);
		void alignHorizontal(double offset, double yaw);
		double map(double val, double x, double y, double j, double k);

		void resetAllEncoders();
		void updateAllEncoders();
		SwerveDrive();
		~SwerveDrive();
};
#endif