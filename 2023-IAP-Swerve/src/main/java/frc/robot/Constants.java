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
        public static final double driveGearRatio = 8.14; // For SDS MK4i module
        public static final double turnGearRatio = 150.0/7.0; // For SDS MK4i module
        public static final double CANCoderGearRatio = 1.0; // Direct measurement

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0); // Assuming SDS module
        
        // These can be safely adjusted without adjusting discrete
        public static final double maxChassisTranslationalSpeed = Units.feetToMeters(12.0); // Assuming L1 swerve
        public static final double maxLinearVelocityMeters = Units.feetToMeters(12.0); // Assuming L1 swerve

        public static final double trackWidthX = Units.inchesToMeters(27.5); // 27.5 inch
        public static final double trackWidthY = Units.inchesToMeters(27.5); // 27.5 inch
        public static final double hypotenuse = Math.sqrt(Math.pow(trackWidthX / 2.0, 2) + Math.pow(trackWidthY / 2.0, 2));

        public static final double maxChassisAngularVelocity = Math.PI * 2.0 * 1.25; // A decent number but not fast enough

        // Joystick deadband for no accidental movement
        public static final double deadBand = 0.1;
    }

    public static final class ModuleConstants {

        // In rotations
        public static final double drivingEncoderPositionFactor = (Math.PI * Constants.SwerveConstants.wheelDiameterMeters) / Constants.SwerveConstants.driveGearRatio;
        
        // In RPM
        public static final double velocityPositionFactor = ((Math.PI * Constants.SwerveConstants.wheelDiameterMeters) / Constants.SwerveConstants.driveGearRatio) / 60.0;


        public static final double turningEncoderPositionFactor = (2 * Math.PI) / Constants.SwerveConstants.turnGearRatio; // radians
        public static final double turningEncoderVelocityFactor = (2 * Math.PI) / Constants.SwerveConstants.turnGearRatio / 60.0; // radians per second


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
        public static final double maxFreeWheelSpeedMeters = (maxFreeSpeed * Math.PI * Constants.SwerveConstants.wheelDiameterMeters) / Constants.SwerveConstants.driveGearRatio;
        // Unit for FF: Motor power / meters/s
        // Calculating it out: 1/3.709 = 0.26958125317 power per meters/second
        // If we want to go to the max speed of 3.709, then multiply velocity error by this constant
        // I.e. 3.709 * 0.2695 ~= 1.0
        public static final double drivekF = 1.0/maxFreeWheelSpeedMeters;

        // We don't know how to calculate this yet :)
        public static final double turnkP = 0.0;
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;

        public static final int driveCurrentLimit = 50;
        public static final int turnCurrentLimit = 20;
    }

}
