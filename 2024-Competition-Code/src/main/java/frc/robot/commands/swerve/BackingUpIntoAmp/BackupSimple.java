package frc.robot.commands.swerve.BackingUpIntoAmp;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIO;


public class BackupSimple extends Command {
    private final SwerveDrive swerveDrive;
    SwerveModuleIO module; //Any one works
    boolean backedUp;

    double targetDisplacement = Units.Meters.convertFrom(  3, Units.Inches);
    double initialDisplacement;
    double currentDisplacement;

    public BackupSimple(SwerveDrive swerveDrive, SwerveModuleIO module) {
        this.swerveDrive = swerveDrive;
        addRequirements(this.swerveDrive);
    }

    @Override
    public void initialize() {
      backedUp = false;
      initialDisplacement = module.getPosition().distanceMeters;
      //Wheel diameter is 4 inches

    }

    @Override
    public void execute() {
       currentDisplacement = module.getPosition().distanceMeters;
       swerveDrive.drive(new Translation2d(-0.3, 0),0 , false, true);
       if (initialDisplacement-currentDisplacement == targetDisplacement){
           backedUp = true;
       }
    }

    @Override
    public boolean isFinished() {
        return backedUp;
    }

    @Override
    public void end(boolean interrupted) {
         swerveDrive.drive(new Translation2d(0,0),0,false, false);
    }
}
