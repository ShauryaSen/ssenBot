package frc.robot.commands.MotionControlDemos;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Date;
import java.text.SimpleDateFormat;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PForwardCommand extends CommandBase{
    
    private final Drivetrain drivetrain;
    private final double setPoint;
    private final double kP;
    private double rawMotorOutput;
    private double error;

    // the pov writer
    private PrintWriter writer;


    // auton that goes forward a specified amount using kP value
    public PForwardCommand(double setPoint, Drivetrain drivetrain, double kP) {
        this.setPoint = setPoint;
        this.drivetrain = drivetrain;
        this.kP = kP;

        try {
            String time = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date());
            this.writer = new PrintWriter("src/main/java/frc/robot/subsystems/Graphs/" + time + ".csv");
        } catch (FileNotFoundException e){
            e.printStackTrace();
        }

        // set up columns
        writer.print("error, ");
        writer.print("MotorOutput (kP * error), ");
        writer.print("Distance, ");
        writer.print("kP, ");
        writer.print("SetPoint, ");
        writer.print("\n, ");
        
        

        

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

        // log
        writer.print(error + ", ");
        writer.print(rawMotorOutput + ", ");
        writer.print(drivetrain.getDistance() + ", ");
        writer.print(kP + ", ");
        writer.print(this.setPoint + ", ");
        writer.print("\n, ");

    }

    @Override
    public boolean isFinished() {
        return error == 0;
        // return Math.abs(rawMotorOutput) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        writer.flush();
        drivetrain.stop();
    }
}
