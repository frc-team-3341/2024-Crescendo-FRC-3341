// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climbSparkMax = new CANSparkMax(Constants.ClimberConstants.extPort, MotorType.kBrushless);
  SparkLimitSwitch forwardLimit;
  SparkLimitSwitch reverseLimit;

  SparkPIDController pid;
  RelativeEncoder encoder;

  public boolean override = true;

  public Climber() {
    forwardLimit = climbSparkMax.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    forwardLimit.enableLimitSwitch(true);

    reverseLimit = climbSparkMax.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    reverseLimit.enableLimitSwitch(true);

    encoder = climbSparkMax.getEncoder();

    encoder.setPositionConversionFactor(Constants.ClimberConstants.climberConversionFactor);
    encoder.setVelocityConversionFactor(Constants.ClimberConstants.velocityConversionFactor);

    pid = climbSparkMax.getPIDController();
    pid.setP(Constants.ClimberConstants.climbkP);
    pid.setI(Constants.ClimberConstants.climbkI);
    pid.setD(Constants.ClimberConstants.climbkD);
  }

  public boolean getOverride() {
    return override;
  }

  public void setOverride(boolean o) {
    override = o;
  }

  public void extendArmWithPower(double power) {
    climbSparkMax.set(power);
  }

  public void extendArmWithVelocity(double velocity) {
    if (forwardLimit.isPressed() | reverseLimit.isPressed()) {
      pid.setReference(0.0, ControlType.kVelocity);
    } else {
      pid.setReference(velocity, ControlType.kVelocity);
    }

  }

  public double getArmPositionInMeters() {
    return encoder.getPosition();
  }

  public double getArmVelocityInMeters() {
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {

    if (forwardLimit.isPressed()) {
      encoder.setPosition(0);
    }

    if (reverseLimit.isPressed()) {
      encoder.setPosition(Constants.ClimberConstants.maxExtensionLimit);
    }

    extendArmWithPower(RobotContainer.getIntakeJoy().getY());

    forwardLimit.enableLimitSwitch(!override);
    reverseLimit.enableLimitSwitch(!override);

    SmartDashboard.putBoolean("forward Limit", forwardLimit.isPressed());
    SmartDashboard.putBoolean("reverse Limit", reverseLimit.isPressed());
    SmartDashboard.putNumber("climber position", encoder.getPosition());
  }
}