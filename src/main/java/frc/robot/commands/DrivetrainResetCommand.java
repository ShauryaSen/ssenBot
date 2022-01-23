package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Simple Command that calls the reset command on the drivetrain
 */
public class DrivetrainResetCommand extends InstantCommand {
    
    Drivetrain drivetrain;  

    public DrivetrainResetCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }

}
