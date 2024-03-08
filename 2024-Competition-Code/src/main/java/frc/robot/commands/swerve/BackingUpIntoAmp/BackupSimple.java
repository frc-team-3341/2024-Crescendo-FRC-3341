package frc.robot.commands.swerve.BackingUpIntoAmp;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;


public class BackupSimple extends Command {
    private final SwerveDrive swerveDrive;
    boolean backedUp;
    double timer; //Timer (in ms)
    double distanceMeters = Units.Meters.convertFrom(3, Units.Inches);

    public BackupSimple(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(this.swerveDrive);
    }

    @Override
    public void initialize() {
      backedUp = false;
      timer = 0;
    }

    @Override
    public void execute() {
        timer += 20;
        //Runs every 20ms
        //Should be run 50 times until it stops
        //swerveDrive.drive happens in meters/second
       swerveDrive.drive(new Translation2d(0, -distanceMeters),0 , false, false);

       if (timer >= 1000){backedUp = true;}
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
