package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision.*;
import frc.robot.subsystems.swerve.SwerveDrive;


public class TargetAprilTag extends Command {
    private PhotonVision photonVision;
    private SwerveDrive swerveDrive;
    public XboxController controller;

    public float[] threshold = {1,1};
    // {rotation threshold (Degrees), centering threshold (meters)}
    public double ZAngle;
    public double XVal;
    public boolean rotationAligned = false;
    public boolean centered = false;

    public TargetAprilTag(PhotonVision photonVision, SwerveDrive swerveDrive) {
        this.photonVision = photonVision;
        this.swerveDrive = swerveDrive;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.photonVision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TESTING Procedure #1 Test to see if this code works and infinitely moves the robot to the left
        //TESTING Procedure: Test the centering and rotation independently, then test them together, then add swerve manual control
        if (photonVision.targetExists() && controller.getLeftBumperPressed()){
            while (!centered){
                XVal = photonVision.getXOffset();
                swerveDrive.drive(new Translation2d(-1, 0), 0, true, false);
                //Move toward center (left or right)
                if ( (-threshold[1] + XVal) <= 0 && 0 <= (threshold[1] + XVal) ){
                    centered = true;
                    break;
                }
            }


//            while (!rotationAligned){
//                ZAngle = Math.abs(photonVision.getZAngle());
//                //Swerve turn (move right if val is > 180 and to the left if val is < 180)
//
//                if ( (-threshold[0] + ZAngle) <= 180 && 180 <= (threshold[0] + ZAngle) ){
//                    rotationAligned = true;
//                    break;
//                }
//            }
            //Using swerve autonomous functions we can move toward the center of the April Tag
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
