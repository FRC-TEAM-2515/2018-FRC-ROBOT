/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2515.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends IterativeRobot {
	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
	private static final double kVoltsPerDegreePerSecond = 0.0128;
	
	private static final int kFrontLeftChannel = 2;
	private static final int kRearLeftChannel = 3;
	private static final int kFrontRightChannel = 1;
	private static final int kRearRightChannel = 0;
	private static final int kGyroPort = 0;
	private static final int kJoystickChannel = 0;

	private MecanumDrive m_robotDrive;
	private AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
	private Joystick m_stick;

	@Override
	public void robotInit() {
		Spark frontLeft = new Spark(kFrontLeftChannel);
		Spark rearLeft = new Spark(kRearLeftChannel);
		Spark frontRight = new Spark(kFrontRightChannel);
		Spark rearRight = new Spark(kRearRightChannel);

		// Invert the left side motors.
		// You may need to change or remove this to match your robot.
		frontLeft.setInverted(true);
		rearLeft.setInverted(true);

		m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

		m_stick = new Joystick(kJoystickChannel);

		m_gyro.setSensitivity(kVoltsPerDegreePerSecond);

		CameraServer.getInstance().startAutomaticCapture();

	}

	@Override
	public void teleopPeriodic() {
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		m_robotDrive.driveCartesian(m_stick.getX(), m_stick.getY(),
				m_stick.getZ(), m_gyro.getAngle());
	}
}
