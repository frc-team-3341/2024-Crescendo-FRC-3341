package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.photonvision.*;
import frc.robot.subsystems.swerve.SwerveDrive;


public class TargetAprilTag extends Command {
    private PhotonVision photonVision;
    private SwerveDrive swerveDrive;

    public double[] threshold = {1.0,0.05};
    // {rotation threshold (Degrees), centering threshold (meters)}
    public double ZAngle;
    public double XVal;
    public int moveDirection = 1;
    //Positive means move left, negative means move right
    public int rotationDirection = 1;
    // Positive means clockwise, negative means counter-clockwise
    public boolean centered = false;
    public boolean rotationAligned = false;


    public TargetAprilTag(PhotonVision photonVision, SwerveDrive swerveDrive) {
        this.photonVision = photonVision;
        this.swerveDrive = swerveDrive;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.photonVision, this.swerveDrive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TESTING Procedure #1 Test to see if this code works and infinitely moves the robot to the left
        //TESTING Procedure: Test the centering and rotation independently, then test them together, then add swerve manual control
        if (photonVision.targetExists()) {

            while (!rotationAligned){
                ZAngle = photonVision.getZAngle();
                if (ZAngle > 0){
                    rotationDirection = -1;
                }else{
                    rotationDirection = 1;
                }
                //Swerve turn (move right if val is > 180 and to the left if val is < 180) [OLD] ??
                swerveDrive.drive(new Translation2d(0,0), 5 * rotationDirection, false, false);
                if ((-threshold[0] <= 0) && (0 <= threshold[0]) ){
                    rotationAligned = true;
                    swerveDrive.drive(new Translation2d(0,0), 0, false, false);
                    break;
                }
            }

            while (!centered) {
                XVal = photonVision.getXOffset();
                if (XVal > 0) {
                    moveDirection = -1;
                } else {
                    moveDirection = 1;
                }
                //positive y is left, negative y is right
                swerveDrive.drive(new Translation2d(0, 0.1 * moveDirection), 0, false, false);
                if ((-threshold[1] <= XVal) && (XVal <= threshold[1])) {
                    centered = true;
                    swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
                    break;
                }
            }
            photonVision.aligned = (centered && rotationAligned);
            photonVision.robotAligning = !(centered && rotationAligned);
        }
    }

    @Override
    public boolean isFinished() {
        return photonVision.aligned;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new Translation2d(0,0), 0, false, false);
    }
}
