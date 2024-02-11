// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.video.SparsePyrLKOpticalFlow;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climbSparkMax = new CANSparkMax(Constants.OperatorConstants.extPort, MotorType.kBrushless);
  SparkLimitSwitch forwardLimit;
  SparkLimitSwitch reverseLimit;
  RelativeEncoder encoder;
  
  public static boolean override = true;

  public Climber() {
    forwardLimit = climbSparkMax.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    forwardLimit.enableLimitSwitch(true);

    reverseLimit = climbSparkMax.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    encoder = climbSparkMax.getEncoder();

    encoder.setPositionConversionFactor(Constants.climberConversionFactor);
    encoder.setVelocityConversionFactor(Constants.velocityConversionFactor);
  }


  public static boolean getOverride(){
      return override;
    }
  public void setOverride(boolean o){
      override = o;
    }
  
  //joystick
  public void extendArm(double power){
    power = RobotContainer.getJoy().getY();
    climbSparkMax.set(power);
  }

  @Override
  public void periodic() {
    if (RobotContainer.getJoy().getRawButtonReleased(Constants.ButtonMap.manualOverride)) {
      override = !override;
    }

    if(forwardLimit.isPressed()){
      encoder.setPosition(0);
    }
    if(reverseLimit.isPressed()){
      encoder.setPosition(Constants.maxExtension); 
    }
  }
}
