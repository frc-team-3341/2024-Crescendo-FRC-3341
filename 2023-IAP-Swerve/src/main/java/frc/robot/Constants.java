// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final class SwerveConstants {
        // These can be safely adjusted without adjusting discrete
        // Some fudge factor is needed for safety while translating + rotating
        public static final double maxChassisTranslationalSpeed = 0.3; // Assuming L1 swerve
        public static final double maxWheelLinearVelocityMeters = ModuleConstants.maxFreeWheelSpeedMeters - 0.2; // Assuming L1 swerve
        public static final double maxChassisAngularVelocity = Math.PI * 2.0; // A decent number but not fast enough

        public static final double trackWidthX = Units.inchesToMeters(27.5); // 27.5 inch
        public static final double trackWidthY = Units.inchesToMeters(27.5); // 27.5 inch
        public static final double hypotenuse = Math.sqrt(Math.pow(trackWidthX, 2) + Math.pow(trackWidthY, 2));

        // Joystick deadband for no accidental movement
        public static final double deadBand = 0.1;

        // Convenient array of module CAN IDs
        // Convention of first array: Front Left, Front Right, Back Left, Back Right
        public static final int[][] moduleCANIDs = {{1, 2, 9}, {3, 4, 10}, {5, 6, 11}, {7, 8, 12}};

        // Initially 0 until we calibrate the modules 12/9
        public static final double[] moduleAngleOffsets = {306.5, 187.6, 109.4, 85.2};

        public static final boolean[] moduleInverts = {false, true, false, true};
    }

    public static final class ModuleConstants {
        
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0); // Assuming SDS module
        
        public static final double driveGearRatio = 8.14; // For SDS MK4i module
        public static final double turnGearRatio = 150.0/7.0; // For SDS MK4i module
        public static final double CANCoderGearRatio = 1.0; // Direct measurement

        // In rotations
        public static final double drivingEncoderPositionFactor = (Math.PI * wheelDiameterMeters) / driveGearRatio;
        
        // In RPM
        public static final double velocityPositionFactor = ((Math.PI * wheelDiameterMeters) / driveGearRatio) / 60.0;

        public static final double turningEncoderPositionFactor = (2 * Math.PI) / turnGearRatio; // radians
        public static final double turningEncoderVelocityFactor = (2 * Math.PI) / turnGearRatio / 60.0; // radians per second

        // Guessed kP
        public static final double drivekP = 0.01;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        // See REV: https://motors.vex.com/other-motors/neo
        // Max free speed in RPM originally, converted to RPS native unit
        public static final double maxFreeSpeed = 5676.0 / 60.0;
        // Unit for this: meters/s
        // Calculating it out:
        // 94.6 RPS * pi * 0.1016 m / 8.14 gearing = 3.7094567527 meters / s = 12.1701337 feet / s
        // Therefore, this max wheel free speed works (compared to empirical MK4i free speed)
        public static final double maxFreeWheelSpeedMeters = (maxFreeSpeed * Math.PI * wheelDiameterMeters) / driveGearRatio;
        // Unit for FF: Motor power / meters/s
        // Calculating it out: 1/3.709 = 0.26958125317 power per meters/second
        // If we want to go to the max speed of 3.709, then multiply velocity error by this constant
        // I.e. 3.709 * 0.2695 ~= 1.0
        public static final double drivekF = 1.0/maxFreeWheelSpeedMeters;

        // We don't know how to calculate this yet :)
        public static final double turnkP = 0.00;
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;

        public static final int driveCurrentLimit = 40;
        public static final int turnCurrentLimit = 20;

    }

}
