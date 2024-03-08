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


public class photonvision extends SubsystemBase {
    public PhotonCamera camera;
    public static Transform3d targetPos;

    public static PhotonPoseEstimator poseEstimator;
    public static boolean[] alignData;

    public photonvision(PhotonCamera camera) {
        this.camera = camera;
        PortForwarder.add(5800, "photonvision", 5800);
        this.camera.setPipelineIndex(0);
    }

    public boolean targetExists(){
        PhotonPipelineResult result = this.camera.getLatestResult();
        return result.hasTargets();
    }

    public Transform3d getTargetData() {
        PhotonPipelineResult result = this.camera.getLatestResult();
        return result.getBestTarget().getBestCameraToTarget();
    }

    public double getZAngle(){
        return (this.targetExists() ? Math.toDegrees(targetPos.getRotation().getAngle()) : 0);
        //Z Angle can determine if the camera is flat to the april tag
    }

    public double getXOffset(){
        return (this.targetExists() ? targetPos.getY() : 0);
        //Y is the left direction but the offset is on a 2D x-axis
    }
    public double getYOffset(){
        return (this.targetExists() ? targetPos.getX() : 0);
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

    public void setAlignData(boolean robotAligning, boolean aligned){
        alignData = new boolean[]{robotAligning, aligned};
    }

    @Override
    public void periodic() {
        targetPos = this.getTargetData();

        SmartDashboard.putNumber("z-angle", this.getZAngle());
        SmartDashboard.putNumber("x-val", this.getXOffset());
        // v could be implemented in swerve subsystem
        SmartDashboard.putBoolean("RobotAligning", alignData[0]);
        SmartDashboard.putBoolean("Aligned", alignData[1]);
        SmartDashboard.putBoolean("TargetExists", this.targetExists());
    }
}