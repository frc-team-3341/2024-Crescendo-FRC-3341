package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.photonvision.PhotonVision;


public class TargetAprilTag extends Command {
    private PhotonVision photonVision;
    public XboxController controller;

    public float[] threshold = {1,1};
    // {rotation threshold (Degrees), centering threshold (meters)}

    public double ZAngle;
    public double xVal;
    public boolean rotationAligned = false;
    public boolean centered = false;

    public TargetAprilTag(PhotonVision photonVision) {
        this.photonVision = photonVision;
//        this.swerveDrive = swerveDrive;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.photonVision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
         if (photonVision.targetExists() && controller.getLeftBumperPressed()){
             while (!centered){
                 xVal = photonVision.getXOffset();
                 //Move toward center (left or right)
                 if ( (-threshold[1] + xVal) <= 0 && 0 <= (threshold[1] + xVal) ){
                     centered = true;
                     break;
                 }
             }

             while (!rotationAligned){
                 ZAngle = Math.abs(photonVision.getZAngle());
                 //Swerve turn (move right if val is > 180 and to the left if val is < 180)

                 if ( (-threshold[0] + ZAngle) <= 180 && 180 <= (threshold[0] + ZAngle) ){
                     rotationAligned = true;
                     break;
                 }
             }
//            Using swerve autonomous functions we can move toward the center of the April Tag
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
