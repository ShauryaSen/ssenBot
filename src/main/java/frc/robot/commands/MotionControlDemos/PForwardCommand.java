package frc.robot.commands.MotionControlDemos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Charts.Grapher;

public class PForwardCommand extends CommandBase{
    
    private final Drivetrain drivetrain;
    private final double setPoint;
    private final double kP;
    private double rawMotorOutput;
    private double error;

    private Grapher grapher;


    // auton that goes forward a specified amount using kP value
    public PForwardCommand(double setPoint, Drivetrain drivetrain, double kP) {
        this.setPoint = setPoint;
        this.drivetrain = drivetrain;
        this.kP = kP;

        this.grapher = new Grapher();


        /** GRAPHER HEADINGS **/
        // grapher.write("error, ");
        // grapher.write("MotorOutput (kP * error)");
        // grapher.write("Distance");
        // grapher.write("kP");
        // grapher.write("SetPoint");
        // grapher.write("\n");

        // reset the encoders
        drivetrain.resetEncoders();

        addRequirements(drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    
    @Override
    public void execute() {
        //SmartDashboard.putNumber("Romi/Setpoint", setPoint);

        error = setPoint - drivetrain.getDistance();
        rawMotorOutput = kP * error;
        drivetrain.arcadeDrive(rawMotorOutput, 0);

        /** GRAPHER LOG **/
        // grapher.write(error);
        // grapher.write(rawMotorOutput);
        // grapher.write(drivetrain.getDistance());
        // grapher.write(kP);
        // grapher.write(this.setPoint);
        // grapher.write("\n");
    }

    @Override
    public boolean isFinished() {
        return error == 0;
        // return Math.abs(rawMotorOutput) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        // grapher.flush();
        drivetrain.stop();
    }
}
