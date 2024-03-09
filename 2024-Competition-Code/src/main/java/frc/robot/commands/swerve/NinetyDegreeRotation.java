package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;


public class NinetyDegreeRotation extends Command {
    private final SwerveDrive swerve;
    public Rotation2d currentRotation; //Used in radians for now
    public double threshold = Units.Radian.convertFrom(1, Units.Degree);
    public NinetyDegreeRotation(SwerveDrive swerve) {
        this.swerve = swerve;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        currentRotation = swerve.getRotation();
    }

    @Override
    public void execute() {
         //Figure out rotation for closest 90 degree movement
        double remainder = currentRotation.getRadians() % (Math.PI/2);

        if (remainder < (Math.PI/4)){ //Add case for rotation being pi/4 exactly
            swerve.drive(new Translation2d(0,0), remainder, false, true);
        }else{
            swerve.drive(new Translation2d(0,0), (Math.PI/2)-remainder, false, true);
        }
    }

    @Override
    public boolean isFinished() {
        double remainder = currentRotation.getRadians() % (Math.PI/2);
        return ( ((Math.PI/2)-threshold <= remainder) || (remainder <= threshold) );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0),0, false,true);
    }
}
