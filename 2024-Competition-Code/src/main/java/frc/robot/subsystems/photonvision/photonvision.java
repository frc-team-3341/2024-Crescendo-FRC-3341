package frc.robot.subsystems.photonvision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVision extends SubsystemBase {
    public PhotonCamera camera;
    public PhotonPipelineResult result;
    public PhotonTrackedTarget target;
    public Transform3d targetPos;
    public boolean hasTarget;
    //Shouldn't be static bc two photonvision subsystems will be used for each camera
    public PhotonPoseEstimator poseEstimator;

    public boolean aligned;
    public boolean robotAligning;

    public PhotonVision(PhotonCamera camera) {
        this.camera = camera;
        PortForwarder.add(5800, "photonvision", 5800);
        this.camera.setPipelineIndex(0);
    }

    public boolean targetExists(){
        result = camera.getLatestResult();
        hasTarget = result.hasTargets();
        return hasTarget;
    }

    public double getZAngle(){
        return hasTarget ? Math.toDegrees(targetPos.getRotation().getAngle()) : 0;
        //Z Angle can determine if the camera is flat to the april tag
    }
    // Makes sure to only execute code if the target exists
    public double getXOffset(){
        return hasTarget ? targetPos.getY() : 0;
        //Y is the left direction but the offset is on a 2D x-axis
    }
    public double getYOffset(){
        return hasTarget ? targetPos.getX() : 0;
        //How far you are to the target. The y distance (api uses x though)
    }

    public Pose3d getFieldCentricPose(){
        //Can load custom layouts as well
        AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        poseEstimator = new PhotonPoseEstimator(aprilTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.PhotonVisionConstants.robotToCamera);
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
//            SmartDashboard.putNumber("z-angle", Math.toDegrees(targetPos.getRotation().getAngle()));
//            SmartDashboard.putNumber("x-val", targetPos.getX());
//            SmartDashboard.putNumber("y-val", targetPos.getY());
        }

        // v could be implemented in swerve subsystem
        SmartDashboard.putBoolean("RobotAligning", robotAligning);
        SmartDashboard.putBoolean("Aligned", aligned);
        SmartDashboard.putBoolean("TargetExists", this.targetExists());
    }
}