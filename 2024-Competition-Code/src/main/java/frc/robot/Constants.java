// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Checks if robot is real or not
    public static boolean isSim = Robot.isSimulation();

    // MODIFY THIS WHEN SWITCHING BETWEEN CHASSIS
    // THIS IS THE FIRST THING YOU SHOULD THINK ABOUT/SEE!!!
    public final static RobotType currentRobot = RobotType.ROBOT_2024_COMPETITION;

    public static class ShooterConstants {
        public final static int upperShooter = 17;
        public final static int lowerShooter = 18;
    }

    public static final class IntakeConstants { // Not CAN ids, DIO port numbers
        public static final int shooterBeamBreak = 1;
        public static final int intakeBeamBreak = 9;
        public static final int feeder = 3;
        public static final int shooter = 4;
        public static final int feederMax = 19;
    }

    public static final class PIDShooterConsts {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class ButtonMap {
        public static final int intakeNote = 2;
    }

    public static final class SwerveConstants {
        // These can be safely adjusted without adjusting discrete
        // Some fudge factor is needed for safety while translating + rotating
        // Max speed is 3.4 m/s
        public static final double maxChassisTranslationalSpeed = ModuleConstants.maxFreeWheelSpeedMeters; // Assuming L1 swerve
        public static final double maxWheelLinearVelocityMeters = ModuleConstants.maxFreeWheelSpeedMeters; // Assuming L1 swerve
        public static final double maxChassisAngularVelocity = Math.PI * 2.0; // A decent number but not fast enough

        public static final double trackWidthX = Units.inchesToMeters(27.5); // 27.5 inch
        public static final double trackWidthY = Units.inchesToMeters(27.5); // 27.5 inch
        public static final double trackWidthHypotenuse = Math.sqrt(Math.pow(trackWidthX, 2) + Math.pow(trackWidthY, 2));

        // Joystick deadband for no accidental movement
        public static final double deadBand = 0.05;

        public static final boolean[] moduleInverts = {false, true, false, true};
    }

    public static final class ModuleConstants {
        
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0); // Assuming SDS module
        
        public static final double driveGearRatio = 8.14; // For SDS MK4i module
        public static final double turnGearRatio = 150.0/7.0; // For SDS MK4i module
        public static final double CANCoderGearRatio = 1.0; // Direct measurement

        // Both of these measurements should be correct
        // In rotations
        public static final double drivingEncoderPositionFactor = (Math.PI * wheelDiameterMeters) / driveGearRatio;
        
        // In RPM
        public static final double drivingEncoderVelocityPositionFactor = ((Math.PI * wheelDiameterMeters) / driveGearRatio) / 60.0;

        public static final double turningEncoderPositionFactor = (2 * Math.PI) / turnGearRatio; // radians
        public static final double turningEncoderVelocityFactor = (2 * Math.PI) / turnGearRatio / 60.0; // radians per second

        // Confirmed working kP!!
        public static final double drivekP = 0.1; // This is good!
        //public static final double drivekP = 0.0;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        // See REV: https://motors.vex.com/other-motors/neo
        // The 5790 value is the correct empirical value from the woodblocks
        // TODO - Might need to be re-calibrated for carpet or concrete
        public static final double maxRPMWoodBlocks = 5790.0;
        public static final double maxRPMCarpet = 5280.0;

        // Max free speed in RPM originally, converted to RPS native unit
        public static final double maxFreeSpeed = maxRPMCarpet / 60.0;
        // Unit for this: meters/s
        // Calculating it out:
        // 94.6 RPS * pi * 0.1016 m / 8.14 gearing = 3.7094567527 meters / s = 12.1701337 feet / s
        // Therefore, this max wheel free speed works (compared to empirical MK4i free speed)
        public static final double maxFreeWheelSpeedMeters = (maxFreeSpeed * Math.PI * wheelDiameterMeters) / driveGearRatio;
        // Unit for FF: Motor power / meters/s
        // Calculating it out: 1/3.709 = 0.26958125317 power per meters/second
        // If we want to go to the max speed of 3.709, then multiply velocity error by this constant
        // I.e. 3.709 * 0.2695 ~= 1.0

        // DO NOT MODIFY THIS UNLESS YOU KNOW WHAT YOU ARE DOING
        public static final double drivekF = 1.0/maxFreeWheelSpeedMeters;

        public static final double turnkP = 0.7; 
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;

        public static final int driveCurrentLimit = 35;
        public static final int turnCurrentLimit = 20;

    }

    public static class ClimberConstants {
      public static final int extPort = 20;
      public static final double climberConversionFactor = (1/3.0) * Units.inchesToMeters(0.2);
      public static final double velocityConversionFactor = (1/3.0)/60.0 * Units.inchesToMeters(0.2);
      public static final double maxExtensionLimit = Units.inchesToMeters(26.55);

      public static final double maxExtensionVelocity = 0.1;

      public static final double climbkP = 0.01;
      public static final double climbkI = 0.0;
      public static final double climbkD = 0.0;

    }
  
    public static final class PhotonVisionConstants {
        // the position of the camera from the center of the robot
        public static final Transform3d robotToCamera = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        public static double cameraHeightMeters = 0;
        public static double targetHeightMeters = 0;
        public static double cameraPitchRadians = 0;
        public static double targetPitchRadians = 0;
    }

}
