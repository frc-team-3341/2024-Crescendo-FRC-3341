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
        public static final double driveGearRatio = 8.16; // For SDS module
        public static final double turnGearRatio = 12.8; // For SDS module

        public static final double wheelDiameterMeters = Units.inchesToMeters(3.0); // Assuming SDS module
        
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
        // Placeholder constants
        public static final double drivingPositionFactor = 1.0;
        public static final double velocityPositionFactor = 1.0;

        public static final double drivekP = 1.0;
        public static final double drivekI = 1.0;
        public static final double drivekD = 1.0;
        public static final double drivekF = 1.0;

        public static final double turnkP = 1.0;
        public static final double turnkI = 1.0;
        public static final double turnkD = 1.0;

        public static final int driveCurrentLimit = 50;
        public static final int turnCurrentLimit = 20;
    }

}
