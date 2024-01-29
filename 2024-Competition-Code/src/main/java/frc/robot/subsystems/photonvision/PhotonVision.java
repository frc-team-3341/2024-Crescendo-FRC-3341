package frc.robot.subsystems.photonvision;


import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVision extends SubsystemBase {
    public PhotonCamera photonCamera;
    public PhotonPipelineResult result;
    public PhotonTrackedTarget target;
    public Transform3d targetPos;
    public static boolean hasTarget;
    public double ZAngle; //Angle of rotation to correct with the AprilTag
    public double XOffset; //the X-offset with the AprilTag


    public PhotonVision() {
        photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        PortForwarder.add(5800, "photonvision", 5800);
        photonCamera.setPipelineIndex(0);
    }

    public boolean targetExists(){
        result = photonCamera.getLatestResult();
        hasTarget = result.hasTargets();
        return hasTarget;
    }

//    public Rotation2d getYawOffset(){
//        Pose2d robotPose;
//        Pose2d targetPose;
//        //getYawToPose provides a delta yaw between the robot and the apriltag
//        //Need to calculate the robot and apriltag pose
//        //We want to move the robot until the yaw is 0 or within a threshold near 0
//        //We are using the built-in yaw for now just to get things working <-- Can change and update later
//        double yaw = this.getYaw();
//        return Rotation2d.fromDegrees(yaw);
//    }
//
//    public double getXOffset1(){
//        double distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
//                Constants.PhotonVisionConstants.cameraHeightMeters,
//                Constants.PhotonVisionConstants.targetHeightMeters,
//                Constants.PhotonVisionConstants.cameraPitchRadians,
//                Constants.PhotonVisionConstants.targetPitchRadians
//        );
//
//        Translation2d dist = PhotonUtils.estimateCameraToTargetTranslation(distanceToTargetMeters, Rotation2d.fromRadians(this.getYaw()));
//        //Should return the x offset from the target (can use y offset to get the distance which can be adjusted for different distances)
//        return dist.getX();
//    }

    public double getZAngle(){
        return targetPos.getRotation().getZ();
    }

    public double getXOffset(){
        return targetPos.getY(); //Y is the left direction but the offset is on a 2D x-axis
    }

    //Z Angle can determine if the camera is flat to the april tag <-- Needs calibration first
    @Override
    public void periodic() {
        if (this.targetExists()){
            target = result.getBestTarget();
            targetPos = target.getBestCameraToTarget();
        }
        SmartDashboard.putBoolean("Target", this.targetExists());

        //Its confusingly described how the cameraToTarget method works and what it gives, so I have put a bunch of targetPosData values set to print for testing. The correct value will become the "Z angle" which can be plugged into the rest of the program
        //We also can use this for the y m which gives the x positioning of the april tag which can once again be plugged in when properly tested
        SmartDashboard.putNumber("targetPosData_1", targetPos.getX());
        SmartDashboard.putNumber("targetPosData_2", targetPos.getY());
        SmartDashboard.putNumber("targetPosData_3", targetPos.getZ());

        SmartDashboard.putNumber("targetPosData_4", targetPos.getRotation().getAngle());
        SmartDashboard.putNumber("targetPosData_5", targetPos.getRotation().getX());
        SmartDashboard.putNumber("targetPosData_6", targetPos.getRotation().getY());
        SmartDashboard.putNumber("targetPosData_7", targetPos.getRotation().getZ());
    }
}