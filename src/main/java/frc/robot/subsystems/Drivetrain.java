// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Motors
  private final Spark leftMotor;
  private final Spark rightMotor; 

  // Encoders
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;  
  
  // Gyro
  private final RomiGyro gyro;

  // Odometer
  private final DifferentialDriveOdometry odometer;

  // Field 

  public Drivetrain() {
    // set up motors
    leftMotor = new Spark(0);
    rightMotor = new Spark(1);
    rightMotor.setInverted(true);

    // set up encoders
    leftEncoder = new Encoder(Constants.LEFT_ENCODER_A, Constants.LEFT_ENCODER_B, false);
    // encoder right inversed 
    rightEncoder = new Encoder(Constants.RIGHT_ENCODER_A, Constants.RIGHT_ENCODER_B, false);

    leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

    // encoders must be set to zero before making DifferentialDriveOdometryy
    resetEncoders();

    // gyro
    gyro = new RomiGyro();

    // odometry
    odometer = new DifferentialDriveOdometry(Constants.START_ANG, Constants.START_POSE);


  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
  
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    tankDrive(speed + rotation, speed - rotation);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  //////// ENCODERS ////////

  /// Left Side ///
  // Get raw encoder count of the left encoder
  public int getRawLeftEncoder() {
    return leftEncoder.get();
  }

  // The distance in meters that the wheel has traveled
  public double getLeftDistance() {
      return leftEncoder.getDistance();
  }

  // The current speed of the left wheel in meters / sec
  public double getLeftVelocity() {
      return leftEncoder.getRate();
  }

  /** Right Side **/
  // Get raw encoder count of the left encoder
  public int getRawRightEncoder() {
      return rightEncoder.get();
  }

  // The distance in meters that the wheel has traveled
  public double getRightDistance() {
      return rightEncoder.getDistance();
  }

  // The current speed of the left wheel in meters / sec
  public double getRightVelocity() {
      return rightEncoder.getRate();
  }

  /** Both Sides **/
  // Current distance in meters that the robot has traveled
  public double getDistance() {
      return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  // The current speed of the robot in meters / sec
  public double getVelocity() {
      return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }

  // Angle of the robot based on the difference in encoder values
  public Angle getEncoderAngle() {
      double diffMeters = getRightDistance() - getLeftDistance();
      return Angle.fromRadians(diffMeters / Constants.TRACK_WIDTH);
  }

  /** Reset **/
  public void resetEncoders() {
      leftEncoder.reset();
      rightEncoder.reset();
  }

  //////// GYRO ////////

  public void getAngle() {}

  //////// ODOMETRY ////////

  public void updateOdometry() {

    

    odometry.update(Rotation2d.fromDegrees(), getLeftDistance(), getRightDistance());
  }

  public void resetOdmetry() {
    
  }
  
  @Override
  public void periodic() {
    updateOdometry();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
  }
}
