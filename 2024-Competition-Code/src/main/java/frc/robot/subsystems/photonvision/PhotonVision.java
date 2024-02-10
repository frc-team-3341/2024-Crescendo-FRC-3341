package frc.robot.subsystems.photonvision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVision extends SubsystemBase {
    public PhotonCamera AprilTagCamera;
    public PhotonCamera NoteCamera;
    public PhotonPipelineResult result;
    public PhotonTrackedTarget target;
    public Transform3d targetPos;
    public static boolean hasTarget;
    public double ZAngle; //Angle of rotation to correct with the AprilTag
    public double XOffset; //the X-offset with the AprilTag
    public PhotonPoseEstimator poseEstimator;


    public PhotonVision() {
        AprilTagCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        NoteCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");

        PortForwarder.add(5800, "photonvision", 5800);

        AprilTagCamera.setPipelineIndex(0);
        NoteCamera.setPipelineIndex(0);
    }

    public boolean targetExists(){
        result = AprilTagCamera.getLatestResult();
        hasTarget = result.hasTargets();
        return hasTarget;
    }

    public double getZAngle(){
        return Math.toDegrees(targetPos.getRotation().getAngle());
    }

    public double getXOffset(){return targetPos.getY();} //Y is the left direction but the offset is on a 2D x-axis

    //Z Angle can determine if the camera is flat to the april tag <-- Needs calibration first

    public Pose3d getFieldCentricPose(){
        //Can load custom layouts as well
        AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // v the position of the camera from the center of the robot
        Transform3d robotToCamera = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        poseEstimator = new PhotonPoseEstimator(aprilTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, AprilTagCamera, robotToCamera);
        return poseEstimator.getReferencePose();
    }

    public void setPose(Pose3d initPose){
        //Only use this during initialization
        poseEstimator.setLastPose(initPose);
    }


    @Override
    public void periodic() {
        if (this.targetExists()){
            target = result.getBestTarget();
            targetPos = target.getBestCameraToTarget();
        }
        SmartDashboard.putBoolean("Target", this.targetExists());

        //Its confusingly described how the cameraToTarget method works and what it gives, so I have put a bunch of targetPosData values set to print for testing. The correct value will become the "Z angle" which can be plugged into the rest of the program
        //We also can use this for the y m which gives the x positioning of the april tag which can once again be plugged in when properly tested
        if (targetPos.getRotation() != null){
            SmartDashboard.putNumber("z-angle", Math.toDegrees(targetPos.getRotation().getAngle()));
        }

        if (targetPos.getTranslation() != null){
            SmartDashboard.putNumber("x-val", targetPos.getX());
            SmartDashboard.putNumber("y-val", targetPos.getY());
        }
    
    }
}
