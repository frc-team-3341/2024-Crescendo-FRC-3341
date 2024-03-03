// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.BackingUpIntoAmp;

/** Helper class for backing-up direction */
public enum BackingDirection {
    NEGATIVE_X(-1), // Back up in negative X direction
    POSITIVE_X(1); // Back up in positive X direction

    public int sign;

    private BackingDirection(int sign) {
        this.sign = (int)Math.signum(sign);
    }

}
