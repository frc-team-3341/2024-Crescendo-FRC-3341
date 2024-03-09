package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import org.opencv.core.Mat;


public class ClosestNinetyDegrees extends Command {
    private final SwerveDrive swerve;
    public Rotation2d currentRotation; //Used in degrees for now
    public double remainder;

    //Test PID
    public PIDController pid = new PIDController(0.1,0,0);
    public ClosestNinetyDegrees(SwerveDrive swerve) {
        this.swerve = swerve;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentRotation = swerve.getRotation();
        //Figure out rotation for closest 90 degree movement
        remainder = -currentRotation.getDegrees() % 90;

        if (remainder < 0){
            remainder = 90-Math.abs(remainder);
        }//Inverts rotation for the case of it being less than 0

        if (remainder <= 45 && remainder > 0){
            //Positive is ccw
            swerve.drive(new Translation2d(0,0), pid.calculate(remainder), false, true);
        }else{
            //Negative is cw
            swerve.drive(new Translation2d(0,0), -pid.calculate(remainder), false, true);
        }
    }

    @Override
    public boolean isFinished() {
        return swerve.inThreshold(1);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0),0, false,true);
    }
}
