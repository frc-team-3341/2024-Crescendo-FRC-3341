// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/*
 * WARNING: MOST CANCODER PHOENIX v5 CLASSES + METHODS HAVE BEEN DEPRECATED. PLEASE FIX :).
 */

/**
 * Implements the SwerveModuleIO interface with two CANSparkMaxes. Uses a
 * CANCoder for turning and PID control using the 20 ms controller provided by
 * WPILib.
 * 
 * @author Aric Volman
 */
public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private CANSparkMax driveSparkMax;
    private CANSparkMax turnSparkMax;

    private SparkPIDController drivePID;
    private SparkPIDController turnPID;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private CANCoder canCoder;

    // Variables to store voltages of motors - REV stuff doesn't like getters
    private double driveVolts = 0.0;
    private double turnVolts = 0.0;

    // Angular offset of module - MAKE SURE CANCODER IS [-180, 180)
    private double offset = 0.0;

    // Object to hold swerve module state
    private SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0.0));

    private int num = 0;

    /**
     * @param num               Module number
     * @param driveID           CAN ID for drive motor
     * @param turnID            CAN ID for turn motor
     * @param turnCANCoderID    CAN ID for CANCoder
     * @param turnEncoderOffset Offset in degrees for module (from -180 to 180)
     * @author Aric Volman
     */
    public SwerveModuleIOSparkMax(int num, int driveID, int turnID, int turnCANCoderID, double turnEncoderOffset,
            boolean invert) {
 
       // TODO - Put config method calls in separate file
       // TIP or TODO - Put config method calls in separate Command object, call via Runnable when needed
       //    --> When needed: when a module's motor could hypothetically reboot during a match

       offset = turnEncoderOffset;
        
       canCoder = new CANCoder(turnCANCoderID);
       driveSparkMax = new CANSparkMax(driveID, MotorType.kBrushless);
       turnSparkMax = new CANSparkMax(turnID, MotorType.kBrushless);

       configCANCoder();
       configDriveMotor(invert);
       configTurnMotor();

       driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
       turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
       turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

       driveEncoder.setPosition(0);
       
       // Offsets the position of the CANCoder via an offset and initializes the turning encoder, placed in proper scope of [-180, 180)
       turnEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition() - offset));
       state.angle = new Rotation2d(turnEncoder.getPosition());

       this.num = num;

    }

    public void configCANCoder() {
        canCoder.configFactoryDefault();

        // Set position of encoder to absolute mode
        canCoder.setPositionToAbsolute();
        // Boot to absolute position
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // Important for matching up the offset of the angle
        // NEED THIS LINE - IF NOT, THEN WRONG OFFSETS WILL BE FOUND
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    }

    public void configDriveMotor(boolean invert) {
        // Reset to defaults
        driveSparkMax.restoreFactoryDefaults();

        // Construct CANSparkMaxes
        // Initialize encoder and PID controller
        driveEncoder = driveSparkMax.getEncoder();
        drivePID = driveSparkMax.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);
        // Set conversion factors
        driveEncoder.setPositionConversionFactor(ModuleConstants.drivingEncoderPositionFactor);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.drivingEncoderVelocityPositionFactor);
        // Set SPARK MAX PIDF
        // More advantageous due to 1 KHz cycle (can ramp up action quicker)
        drivePID.setP(ModuleConstants.drivekP);
        drivePID.setI(ModuleConstants.drivekI);
        drivePID.setD(ModuleConstants.drivekD);
        drivePID.setFF(ModuleConstants.drivekF);
        drivePID.setOutputRange(-1, 1);

        driveSparkMax.setInverted(invert);

        driveSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveSparkMax.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);

        driveSparkMax.enableVoltageCompensation(12.0);
    }

    public void configTurnMotor() {
        // Reset to defaults
        turnSparkMax.restoreFactoryDefaults();

        // Initialize encoder and PID controller
        turnEncoder = turnSparkMax.getEncoder();
        turnPID = turnSparkMax.getPIDController();
        turnPID.setFeedbackDevice(turnEncoder);
        // Set conversion factors
        turnEncoder.setPositionConversionFactor(ModuleConstants.turningEncoderPositionFactor);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.turningEncoderVelocityFactor);
        // Derived from: https://github.com/BroncBotz3481/YAGSL-Configs/tree/main/sds/mk4i/neo
        turnSparkMax.setInverted(true); // This works, because the MK4i is inverted

        // Set SPARK MAX PIDF
        // More advantageous due to 1 KHz cycle (can ramp up action quicker)
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI);

        turnSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnSparkMax.setSmartCurrentLimit(ModuleConstants.turnCurrentLimit);

        turnPID.setP(Constants.ModuleConstants.turnkP);
        turnPID.setI(Constants.ModuleConstants.turnkI);
        turnPID.setD(Constants.ModuleConstants.turnkD);
        turnPID.setFF(0);
        turnPID.setOutputRange(-1, 1);
        
    }

    public double getTurnPositionInRad() {
        // Position should be already offsetted in constructor
        // Modulus places measurement of relative encoder in absolute coordinates (-180 to 180 scope)
        // Modulus is needed if the wheel is spun too much relative to the encoder's starting point
        return MathUtil.angleModulus(turnEncoder.getPosition());
    }

    public void setDesiredState(SwerveModuleState state) {
        // Optimize state so that movement is minimized
        // Should be fine optimizing with PID strategy of -180 to 180 scope, not 0 to 360 scope
        // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/src-html/edu/wpi/first/math/kinematics/SwerveModuleState.html#line.65
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurnPositionInRad()));

        // Cap setpoints at max speeds for safety
        state.speedMetersPerSecond = MathUtil.clamp(state.speedMetersPerSecond,
                -Constants.ModuleConstants.maxFreeWheelSpeedMeters, Constants.ModuleConstants.maxFreeWheelSpeedMeters);

        // Set reference of drive motor's PIDF internally in SPARK MAX
        // This automagically updates at a 1 KHz rate
        drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        // Set setpoint of WPILib PID controller for turning
        // The built-in position wrapping should handle any discontinuity around the -180 to 180 range if needed
        turnPID.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        // Set internal state as passed-in state
        this.state = state;

    }

    public SwerveModuleState getDesiredState() {
        // Returns module state
        return this.state;
    }

    public SwerveModuleState getActualModuleState() {
        double velocity = this.driveEncoder.getVelocity();
        double rotation = this.getTurnPositionInRad();
        return new SwerveModuleState(velocity, Rotation2d.fromRadians(rotation));
    }

    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
        this.driveVolts = volts;
    }

    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
        this.turnVolts = volts;
    }

    public void setDriveBrakeMode(boolean enable) {
        if (enable) {
            driveSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            driveSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setTurnBrakeMode(boolean enable) {
        if (enable) {
            turnSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            turnSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }

    public SwerveModulePosition getPosition() {
        double position = driveEncoder.getPosition();
        double rotation = this.getTurnPositionInRad();
        return new SwerveModulePosition(position, new Rotation2d(rotation));
    }

    public void resetEncoders() {
        // Resets only drive encoder
        driveEncoder.setPosition(0.0);
    }

    public void updateTelemetry() {

        // ESSENTIAL TELEMETRY
        // Show turning position and setpoints
        SmartDashboard.putNumber("Turn Pos Deg #" + this.num, Units.radiansToDegrees(getTurnPositionInRad()));
        // Show driving velocity
        SmartDashboard.putNumber("Drive Vel #" + this.num, driveEncoder.getVelocity());

        // NON-ESSENTIAL TELEMETRY
        if (Constants.currentRobot.enableSwerveMotorTelemetry) {

            // Show driving velocity setpoints
            SmartDashboard.putNumber("Setpoint Drive Vel #" + this.num, state.speedMetersPerSecond);

            // Show turning position and setpoints
            SmartDashboard.putNumber("Radian Turn Pos #" + num, getTurnPositionInRad());
            SmartDashboard.putNumber("Rad Setpoint Turn Pos #" + this.num, state.angle.getRadians());
            SmartDashboard.putNumber("Setpoint Turn Pos Deg #" + this.num, Units.radiansToDegrees(state.angle.getRadians()));

            // Get RPMs
            SmartDashboard.putNumber("Turn RPM #" + this.num, (turnEncoder.getVelocity() / 360.0) * 60.0);
            SmartDashboard.putNumber("Drive RPS #" + this.num,
                    driveEncoder.getVelocity() / Constants.ModuleConstants.drivingEncoderPositionFactor);
            SmartDashboard.putNumber("CANCoder #" + this.num, canCoder.getAbsolutePosition());
            SmartDashboard.putNumber("Drive Motor Voltage #" + this.num, driveSparkMax.getAppliedOutput());

            // Output of driving
            SmartDashboard.putNumber("Turn Volts #" + this.num, this.turnVolts);
            SmartDashboard.putNumber("Drive Volts #" + this.num, this.driveVolts);

            // Get Wheel Displacement
            SmartDashboard.putNumber("Wheel Displacement #" + this.num, getPosition().distanceMeters);
        
        }
    

          
    }

    public int getNum() {
        return num;
    }

}