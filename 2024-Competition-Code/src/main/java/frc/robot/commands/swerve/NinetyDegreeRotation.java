package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;


public class NinetyDegreeRotation extends Command {
    private final SwerveDrive swerve;
    public Rotation2d currentRotation; //Used in degrees for now
    public double remainder;
    public NinetyDegreeRotation(SwerveDrive swerve) {
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

        if ((remainder <= 45 && remainder >= 0) || (Math.abs(remainder) > 45  && remainder <= 0)){ //Add case for rotation being pi/4 exactly
            swerve.drive(new Translation2d(0,0), 1, false, true);
        }else{
            swerve.drive(new Translation2d(0,0), -1, false, true);
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
