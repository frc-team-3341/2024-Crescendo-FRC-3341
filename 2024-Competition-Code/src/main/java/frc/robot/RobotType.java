// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Special class that defines the type of robot used (sim, Competition, IAP, etc.) */
public enum RobotType {
    // 2023 IAP Robot
    ROBOT_2023_IAP_SLOTH(
            new double[] { -62.51, -179.82, 108.11, 82.62}, // Offsets for IAP chassis
            new int[][] { { 1, 2, 9 }, { 3, 4, 10 }, { 5, 6, 11 }, { 7, 8, 12 } }, // CAN IDs for IAP chassis
            true, // Enable data log or not
            true, // Enable XBox driving or not
            true, // Drive according to FMS Alliance
            false, // Invert speed controls for Right Trigger
            false, // Disable climber
            false // Disable shooter
    ),
    // 2024 Competition Robot
    ROBOT_2024_COMPETITION(
            new double[] { -12.21, -121.29, -133.154, -40.97 }, // Offsets for Competition Bot
            // CAN IDs are slightly different - REV PDH is at CAN ID 9
            new int[][] { { 1, 2, 10 }, { 3, 4, 11 }, { 5, 6, 12 }, { 7, 8, 13 } }, // CAN IDs for Competition Bot
            true, // Enable data log or not
            true, // Enable XBox driving or not
            true, // Drive according to FMS Alliance
            true, // Invert speed controls for Right Trigger
            true, // Enable climber
            true // Enable shooter
    ),
    ROBOT_2024_SIMULATION(
            // These arguments are here if accidentally used
            new double[] { -12.21, -121.29, -133.154, -40.97 }, // Offsets for Competition Bot - NOT USED FOR SIM
            // CAN IDs are slightly different - REV PDH is at CAN ID 9
            new int[][] { { 1, 2, 10 }, { 3, 4, 11 }, { 5, 6, 12 }, { 7, 8, 13 } }, // CAN IDs for Competition Bot - NOT USED FOR SIM
            false, // Enable data log or not
            false, // Enable XBox driving or not
            false, // Drive according to FMS Alliance
            true, // Invert speed controls for Right Trigger
            false, // Disable climber
            false // Disable shooter
    );

    /**
     * Angular offsets of CANCoders
     */
    public double[] moduleAngleOffsets;

    /**
     * CAN IDs for each motor and CANCoder of each module
     */
    public int[][] moduleCANIDs;

    /** 
     * If we need to data log or not <p>
    Works within simulation! <p>
    False : not data log <p>
    True : will data log <p>
    */
    public boolean dataLogEnabled;

    /**
     Checks if using XBox or keyboard <p>
     False : keyboard <p>
     True : Xbox <p>
    */
    public boolean xboxEnabled;

    /** Whether to set alliance for teleop driving or not */
    public boolean allianceEnabled;

    /** Whether to invert the right trigger's function
    * False: Robot is default slow, Right Trigger speeds up robot
    * True: Robot is default fast, Right Trigger slows down robot  */
    public boolean invertSpeedControl;

    public boolean enableClimber;

    public boolean enableShooter;

    /** Special constructor for enumerator -> Helps us easily switch between both chassis (REDEPLOYING ONLY FOR NOW) */
    private RobotType(double[] offsets, int[][] ids, boolean dataLogEnabled, boolean xboxEnabled, boolean allianceEnabled, boolean invertSpeedControl, boolean enableClimber, boolean enableShooter) {
        this.moduleAngleOffsets = offsets;
        this.moduleCANIDs = ids;
        this.dataLogEnabled = dataLogEnabled;
        this.xboxEnabled = xboxEnabled;
        this.allianceEnabled = allianceEnabled;
        this.invertSpeedControl = invertSpeedControl;
        this.enableClimber = enableClimber;
        this.enableShooter = enableShooter;
    }

}