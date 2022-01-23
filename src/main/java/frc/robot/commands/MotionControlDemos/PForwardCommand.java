package frc.robot.commands.MotionControlDemos;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PForwardCommand extends CommandBase{
    
    private final Drivetrain drivetrain;
    private final double setPoint;
    private final double kP;
    private double rawMotorOutput;
    private double error;


    // auton that goes forward a specified amount using kP value
    public PForwardCommand(double setPoint, Drivetrain drivetrain, double kP) {
        this.setPoint = setPoint;
        this.drivetrain = drivetrain;
        this.kP = kP;
        System.out.println(error);

        // reset the encoders
        drivetrain.resetEncoders();

        addRequirements(drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    
    @Override
    public void execute() {
        error = setPoint - drivetrain.getDistance();
        rawMotorOutput = kP * error;
        drivetrain.arcadeDrive(rawMotorOutput, 0);
    }

    @Override
    public boolean isFinished() {
        //return error == 0;
        return Math.abs(rawMotorOutput) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
