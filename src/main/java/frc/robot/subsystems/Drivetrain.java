// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  Spark leftMotor;
  Spark rightMotor; 
  

  public Drivetrain() {
    leftMotor = new Spark(0);
    rightMotor = new Spark(1);
    rightMotor.setInverted(true);
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
