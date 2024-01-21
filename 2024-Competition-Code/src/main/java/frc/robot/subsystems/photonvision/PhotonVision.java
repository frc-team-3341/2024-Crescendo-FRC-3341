package frc.robot.subsystems.photonvision;


import edu.wpi.first.math.geometry.Rotation2d;
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
    public static double pose;
    public static Constants c = new Constants();
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
    public double getYaw(){
        //Degrees away from the target
        return yaw;
    }
    public double getPitch(){
        return pitch;
    }
    public double getArea(){
        return area;
    }
    public double getSkew(){
        return skew;
    }
//    public double getPose(){
//        PhotonUtils.calculateDistanceToTargetMeters()
//    }

    //Z Angle can determine if the camera is flat to the april tag <-- Needs calibration first
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