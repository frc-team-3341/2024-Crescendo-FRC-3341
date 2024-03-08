package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.photonvision.*;
import frc.robot.subsystems.swerve.SwerveDrive;


public class TargetAprilTag extends Command {
    private photonvision photonVision;
    private SwerveDrive swerveDrive;

    public double[] threshold = {176.0,0.1};
    // {rotation threshold (Degrees), centering threshold (meters)}
    public static double XVal;
    public static int moveDirection = 1;
    //Positive means move left, negative means move right
    public static boolean aligned;
    public static boolean robotAligning;


    public TargetAprilTag(photonvision photonVision, SwerveDrive swerveDrive) {
        this.photonVision = photonVision;
        this.swerveDrive = swerveDrive;
        addRequirements(this.photonVision, this.swerveDrive);
    }

    @Override
    public void initialize() {
        aligned = false;
        robotAligning = false;
    }

    @Override
    public void execute() {

        if (photonVision.targetExists()) {
            photonVision.setAlignData(robotAligning, aligned);

            XVal = photonVision.getXOffset();
            if (XVal > 0) {
                moveDirection = 1;
            } else {
                moveDirection = -1;
            }
            //positive y is ___, negative y is ___
            swerveDrive.drive(new Translation2d(0, 0.1 * moveDirection), 0, false, false);
            robotAligning = true;
            aligned = false;
            if ((-threshold[1] <= XVal) && (XVal <= threshold[1])) {
                robotAligning = false;
                aligned = true;
                swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
            }
        }

    }

    @Override
    public boolean isFinished() {
        return (aligned || !photonVision.targetExists());
    }

    @Override
    public void end(boolean interrupted) {
        //swerveDrive.drive(new Translation2d(0,0), 0, false, false);
        //Don't stop because if you press the button mid-movement, the robot's velocity will freeze to a halt
    }
}
