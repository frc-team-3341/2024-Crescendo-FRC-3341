package frc.robot.subsystems.photonvision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVision extends SubsystemBase {
    public PhotonCamera photonCamera;
    public PhotonPipelineResult result;
    public PhotonTrackedTarget target;
    public static boolean hasTarget;
    public static double yaw;
    public static double pitch;
    public static double area;
    public static double skew;


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

    public double getYaw(){ return yaw; }
    public double getPitch(){
        return pitch;
    }
    public double getArea(){
        return area;
    }
    public double getSkew(){
        return skew;
    }

    public Rotation2d getYawOffset(){
        Pose2d robotPose;
        Pose2d targetPose;
        //getYawToPose provides a delta yaw between the robot and the apriltag
        //Need to calculate the robot and apriltag pose
        //We want to move the robot until the yaw is 0 or within a threshold near 0
        //We are using the built-in yaw for now just to get things working <-- Can change and update later
        double yaw = this.getYaw();
        return Rotation2d.fromDegrees(yaw);
    }

    public double getXOffset(){
        double distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.PhotonVisionConstants.cameraHeightMeters,
                Constants.PhotonVisionConstants.targetHeightMeters,
                Constants.PhotonVisionConstants.cameraPitchRadians,
                Constants.PhotonVisionConstants.targetPitchRadians
        );

        Translation2d dist = PhotonUtils.estimateCameraToTargetTranslation(distanceToTargetMeters, Rotation2d.fromRadians(this.getYaw()));
        //Should return the x offset from the target (can use y offset to get the distance which can be adjusted for different distances)
        return dist.getX();
    }

    //Z Angle can determine if the camera is flat to the april tag <-- Needs calibration first??
    @Override
    public void periodic() {
        if (this.targetExists()){
            target = result.getBestTarget();
            yaw = target.getYaw();
            pitch = target.getPitch();
            area = target.getArea();
            skew = target.getSkew();
            //Skew doesn't work with April-Tags
        }
        SmartDashboard.putBoolean("Target", this.targetExists());
        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Area", area);
        SmartDashboard.putNumber("Skew", skew);
    }
}