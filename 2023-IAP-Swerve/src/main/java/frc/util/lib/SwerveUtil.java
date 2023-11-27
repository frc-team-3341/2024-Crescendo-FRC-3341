// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.lib;

import java.util.ArrayList;
import java.util.stream.Stream;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveModuleIO;

/**
 * A Swerve Utility class to carry out some of the work. Should work like legos! Our subsystem is now 200 lines! Many functions (discretize, telemetry, wrangling, getters/setters) are common for swerve and can be written here.
 * @author Aric Volman
 */
public class SwerveUtil {

    private static double integratedSimAngle = 0.0;

    private static final Translation2d[] translations = new Translation2d[] {
        new Translation2d(Constants.SwerveConstants.trackWidthX / 2.0, Constants.SwerveConstants.trackWidthY / 2.0),
        new Translation2d(Constants.SwerveConstants.trackWidthX / 2.0,
              -Constants.SwerveConstants.trackWidthY / 2.0),
        new Translation2d(-Constants.SwerveConstants.trackWidthX / 2.0,
              Constants.SwerveConstants.trackWidthY / 2.0),
        new Translation2d(-Constants.SwerveConstants.trackWidthX / 2.0,
              -Constants.SwerveConstants.trackWidthY / 2.0) };
    
    /**
    * Accurately draws module poses on SmartDashboard
    * @param modulePositions Array of swerve module angles + displacements
    * @param field Field2d object to display modules on
    * @param pose Pose of robot to use for position calculations
    */
    public static void drawModulePoses(SwerveModulePosition[] modulePositions, Field2d field, Pose2d pose) {
        var translations = getModuleTranslations();
        for (int i = 0; i < modulePositions.length; i++) {
            Rotation2d moduleRot = modulePositions[i].angle;
            Rotation2d relRot = pose.getRotation();
            // Multiply translation with hypotenuse and add this to the pose of the robot
            
            field.getObject("Module" + i).setPose(
            
                pose.getX() + Math.abs(translations[i].getX())
                * relRot.plus(new Rotation2d(Math.PI/4+i*Math.PI/2)).getCos()*Constants.SwerveConstants.hypotenuse, 
                pose.getY() + Math.abs(translations[i].getY())
                * relRot.plus(new Rotation2d(Math.PI/4+i*Math.PI/2)).getSin()*Constants.SwerveConstants.hypotenuse, 
                moduleRot
            );
                
        }
    }

    /**
     * Sets module positions of array moduleIO
     * @param moduleIO Array of module interface class to use (uses .getPosition())
     */
    public static SwerveModulePosition[] setModulePositions(SwerveModuleIO[] moduleIO) {
        SwerveModulePosition[] returnedPositions = new SwerveModulePosition[moduleIO.length];

        // Iterate through module positions and assign 0 values
        for (int i = 0; i < moduleIO.length; i++) {
            returnedPositions[i] = moduleIO[i].getPosition();
        }

        return returnedPositions;
    }

    /**
    * <p><b>Deprecated</b> for AdvantageScope 2024</p>
    * Gets module states as double[] for AdvantageScope compatibility
    * @param states Array of swerve module states (size doesn't matter, preferably 4)
    */
    public static double[] getDoubleStates(SwerveModuleState[] states) {
    
        ArrayList<Double> ret = new ArrayList<Double>();

        for (SwerveModuleState state : states) {
            ret.add(state.angle.getRadians());
            ret.add(state.speedMetersPerSecond);
        }

        Double[] actual = new Double[states.length*2];
        ret.toArray(actual);

        return Stream.of(actual).mapToDouble(Double::doubleValue).toArray();

    }

    /**
    * Credit: WPIlib 2024 + Patribots (Author: Alexander Hamilton). Discretizes a continuous-time chassis speed.
    * 
    * @param speeds Inputed speeds from joysticks
    * @param discretizeConstant Constant that adjusts dx/dt and dy/dt, mulitplied by omega
    */
    public static ChassisSpeeds discretize(ChassisSpeeds speeds, double discretizeConstant) {
        if (!RobotContainer.getSimOrNot()) {
            return speeds;
        }

        double dt = 0.02;

        var desiredDeltaPose = new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * discretizeConstant)); // -5.0

        var twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    /**
     * Adds simulation to the Swerve subsystem. Works together with integration of angle and SwerveModuleIO. Requires the use of the SwerveModuleIO interface.
     * 
     * @param moduleIO Array of module interface class to use
     * @param actualStates Array of swerve module states
     * @param kinematics Object representing kinematics class
     */
    public static void addSwerveSimulation(SwerveModuleIO[] moduleIO, SwerveModuleState[] actualStates, SwerveDriveKinematics kinematics) {
        // Simulate Navx
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

        // Find omega/angular velocity of chassis' rotation
        // Robot oriented speeds, not field oriented
        double omega = kinematics.toChassisSpeeds(actualStates).omegaRadiansPerSecond;

        // Integrate dAngle into angular displacement
        SwerveUtil.integratedSimAngle += 0.02 * omega * (180 / Math.PI); // convert dradians to degrees

        // Set this as gyro measurement
        angle.set(SwerveUtil.integratedSimAngle);

        // Update moduleIO's sim objects with a dt of 0.02
        for (SwerveModuleIO module : moduleIO) {
            module.updateSim();
        }
    }

    /**
    * Get physical positions of wheels on Swerve chassis (half of trackwidth)
    * @return translations Translation2d[] array with several translations
    */
    public static Translation2d[] getModuleTranslations() {
        return translations;
    }
}