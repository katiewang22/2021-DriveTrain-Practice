// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 2 * Math.PI; // one rotation per second
  private static final double IN_TO_M = .0254;
  private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
  private static final double DIAMETER_INCHES = 5.0; // Flex wheels on Everybot
  private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 12.75;
  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  private static final WPI_TalonSRX frontLeftMotor = RobotMap.frontLeftDriveMotor;
  private static final WPI_TalonSRX frontRightMotor = RobotMap.frontRightDriveMotor;
  private static final WPI_TalonSRX backLeftMotor = RobotMap.backLeftDriveMotor;
  private static final WPI_TalonSRX backRightMotor = RobotMap.backRightDriveMotor;

  public DriveSubsystem() {
    resetEncoders();

    frontLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontLeftMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    frontLeftMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
    frontLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
    
    frontRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRightMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    frontRightMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
    frontRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

		frontLeftMotor.set(ControlMode.Follower, backLeftMotor.getDeviceID());
    frontRightMotor.set(ControlMode.Follower, backRightMotor.getDeviceID());

    backLeftMotor.setNeutralMode(NeutralMode.Coast);
    backRightMotor.setNeutralMode(NeutralMode.Coast);

    backLeftMotor.setInverted(true);//false
    frontLeftMotor.setInverted(true);//false
    backRightMotor.setInverted(false); //true 
    frontRightMotor.setInverted(false); //true
  }

  @Override
  public void periodic() {
  }

  public void setModePercentVoltage() {
    frontLeftMotor.set(ControlMode.PercentOutput, 0);
    frontRightMotor.set(ControlMode.PercentOutput, 0);
    backLeftMotor.set(ControlMode.PercentOutput, 0);
    backRightMotor.set(ControlMode.PercentOutput, 0);
  }

  public double distanceTravelledinTicks() {
    return (getBackLeftEncoderPosition() + getBackRightEncoderPosition()) / 2;
  }

  public double getBackLeftEncoderPosition() {
    return backLeftMotor.getSelectedSensorPosition();
  }

  public double getBackRightEncoderPosition() {
    return backRightMotor.getSelectedSensorPosition();
  }

  public double getBackLeftEncoderVelocity() {
    return backLeftMotor.getSelectedSensorVelocity();
  }

  public double getBackRightEncoderVelocity() {
    return backRightMotor.getSelectedSensorVelocity();
  }

  public double getBackLeftEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backLeftVelocityMPS = (backLeftMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    backLeftVelocityMPS = backLeftVelocityMPS * METERS_PER_TICKS;
    return (backLeftVelocityMPS);
  }

  public double getBackRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backRightVelocityMPS = (backRightMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    backRightVelocityMPS = backRightVelocityMPS * METERS_PER_TICKS;
    return (backRightVelocityMPS);
  }

  public double getAverageEncoderVelocityMetersPerSecond() {
    double velocityMPS = (getBackRightEncoderVelocityMetersPerSecond() + getBackRightEncoderVelocityMetersPerSecond())
        * 0.5;
    return (velocityMPS);
  }

  public double leftDistanceTravelledInMeters() {
    double left_dist = getBackLeftEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }

  public double rightDistanceTravelledInMeters() {
    double right_dist = getBackRightEncoderPosition() * METERS_PER_TICKS;
    return right_dist;
  }

  public double distanceTravelledinMeters() {
    // left distance is negative because the encoder value on the 
    // left is negative when dev bot is pushed forward 2/15/20
    // Code Tested on Dev Bot, Works on 2/15/20
    double distanceTravelled = (leftDistanceTravelledInMeters() + rightDistanceTravelledInMeters()) / 2;
    return distanceTravelled;
  }

  public void resetEncoders() {
    backLeftMotor.setSelectedSensorPosition(0);
    backRightMotor.setSelectedSensorPosition(0);
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }
  
  // basic, no frills, no PID, no nothing drivetrain
  public static void drive(double throttle, double rotate) {
    backLeftMotor.set(throttle - rotate);
    backRightMotor.set(throttle + rotate);
  }
  
  public void stop() {
    drive(0, 0);
  }
}