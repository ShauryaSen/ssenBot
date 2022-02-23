// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivetrainDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;
  private final AutoGamepad driver;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public DrivetrainDrive(Drivetrain subsystem, AutoGamepad gamepad) {

        driver = gamepad;
        drivetrain = subsystem;

        addRequirements(subsystem);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = driver.getLeftY();
    double rightX = driver.getRightX();

    if (Math.abs(driver.getLeftY()) < 0.1) {
      leftY = 0.0;
    }
    if (Math.abs(driver.getRightX()) < 0.1) {
      rightX = 0.0;
    }
      drivetrain.arcadeDrive(leftY, rightX);
  }

}
