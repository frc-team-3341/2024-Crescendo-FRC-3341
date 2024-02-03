// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private RelativeEncoder relativeEncoder;
  private RelativeEncoder relativeEncoder2;
  public final CANSparkMax upperShooter = new CANSparkMax(Constants.ShooterConstants.upperShooter, MotorType.kBrushless);
  public final CANSparkMax lowerShooter = new CANSparkMax(Constants.ShooterConstants.lowerShooter, MotorType.kBrushless);
  public double TicksToRPM = 42.0/60.0/100.0;
  public double setPoint;
  private double power = 1000;
  private double lowerPower = 1000;
  private double rpm = 0;
  public SparkPIDController Controller;
  public SparkPIDController lowerController;
  private SimpleMotorFeedforward neoMotorFeedforward = new SimpleMotorFeedforward(Constants.feedForwardConsts.kS, Constants.feedForwardConsts.kV, Constants.feedForwardConsts.kA);
  /** Creates a new Shooter. */
  public Shooter() {
    upperShooter.restoreFactoryDefaults();
    lowerShooter.restoreFactoryDefaults();
    upperShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    upperShooter.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);
    lowerShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lowerShooter.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);
    relativeEncoder = upperShooter.getEncoder();
    relativeEncoder2 = lowerShooter.getEncoder();
    
    Controller = upperShooter.getPIDController();
    Controller.setP(0);
    Controller.setI(0);
    Controller.setD(0);
    //Controller.setFF(1.0/5891.0); decently working
    Controller.setFF(1.0/6100.0);

    lowerController = lowerShooter.getPIDController();
    lowerController.setP(0);
    lowerController.setI(0);
    lowerController.setD(0);
    lowerController.setFF(1.0/5891.0);
    Preferences.initDouble("power", power);
    Preferences.initDouble("Lower Power", lowerPower);
    
    
    
    
    upperShooter.setInverted(true);
    lowerShooter.setInverted(true);
  }

  

  public double getUpperRPM(){
    return relativeEncoder.getVelocity();
  }
  public double getLowerRPM(){
    return relativeEncoder2.getVelocity();
  }

  public void resetlowerShooter(){
    relativeEncoder.setPosition(0);
  }
   public void resetupperShooter(){
    relativeEncoder2.setPosition(0);
  }

  public void setupperSpeed(double setPoint){
    Controller.setReference(setPoint, ControlType.kVelocity);
    /*if (setPoint == 0){
      upperShooter.set(setPoint);
}
    else{ 
      upperShooter.setVoltage(pid.calculate(getUpperRPM(), setPoint));
    }  */
  }
  public void setlowerSpeed(double setPoint){
    lowerController.setReference(setPoint, ControlType.kVelocity);
    //if (setPoint == 0){
     // lowerShooter.set(setPoint);
   // }
   // else{ 
      //lowerShooter.set(controller.calculate(getLowerRPM(),setPoint));
    }
  

  public void setNeoSpeed(double speed){
    upperShooter.setVoltage(speed*12.0);
  }

  @Override
  public void periodic() {
    power = Preferences.getDouble("power", power);
    lowerPower = Preferences.getDouble("lower power", lowerPower);
    //setNeoSpeed(power);
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(5)){
      rpm = 500;
    }
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(6)){
      rpm= 0;
    }
    setupperSpeed(power);
    setlowerSpeed(lowerPower);
    SmartDashboard.putNumber("joyY", RobotContainer.getIntakeJoy().getY());
    SmartDashboard.putNumber("rpm", getUpperRPM());
    // This method will be called once per scheduler run
  }
}