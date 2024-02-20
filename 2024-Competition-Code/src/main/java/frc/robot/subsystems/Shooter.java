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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
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

  public double setPoint;
  private double power = 0;
  private double lowerPower = 0;
  private double intakePower = 0;

  private double upperP = 0.0001;
  private double upperI = 0.000001;
  private double lowerP = 0.0001;
  private double lowerI = 0.000001;

  public SparkPIDController Controller;
  public SparkPIDController lowerController;
  
  private RelativeEncoder intakeEncoder;
  public final CANSparkMax intakeMax = new CANSparkMax(Constants.IntakeConstants.feederMax, MotorType.kBrushless);
  //private double power = 0;
  DigitalInput shooterBeamBreak = new DigitalInput(Constants.IntakeConstants.shooterBeamBreak);
  DigitalInput intakeBeamBreak = new DigitalInput(Constants.IntakeConstants.intakeBeamBreak);  
  // Creates a new Shooter.
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
    Controller.setP(0.0001);
    Controller.setI(0.000001);
    Controller.setD(0);
    //Controller.setFF(1.0/5891.0); decently working
    Controller.setFF(1.0/6100.0);

    lowerController = lowerShooter.getPIDController();
    lowerController.setP(0.0001);
    lowerController.setI(0.000001);
    lowerController.setD(0);
    lowerController.setFF(1.0/6100.0);
    Preferences.initDouble("power", power);
    Preferences.initDouble("Lower Power", lowerPower);
    Preferences.initDouble("intake Power", intakePower);

    Preferences.initDouble("upperPID P", upperP);
    Preferences.initDouble("upperPID I", upperI);
    Preferences.initDouble("lowerPID P", lowerP);
    Preferences.initDouble("lowerPID I", lowerI);

    intakeMax.restoreFactoryDefaults();
    //resetIntakeEncoder();
    intakeMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMax.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);
    intakeEncoder = intakeMax.getEncoder();
    
    
    
    
    upperShooter.setInverted(false); // Positive = shooting out the note
    lowerShooter.setInverted(true); // Positive = shooting out the note
    intakeMax.setInverted(true); // Positive is intaking to shooter
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

  public void resetIntakeEncoder(){
    intakeEncoder.setPosition(0);
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

  public boolean getShooterBeam(){
    return !shooterBeamBreak.get();
  }
   public boolean getIntakeBeam(){
     return !intakeBeamBreak.get();
  }

  public double getIntakeRPM(){
    return intakeEncoder.getVelocity();
  }

  public void setFeedSimple(double setPoint){
    intakeMax.set(setPoint);
  }
  

  @Override
  public void periodic() {
    
    //power = Preferences.getDouble("power", power);
    //lowerPower = Preferences.getDouble("lower power", lowerPower);
    //intakePower = Preferences.getDouble("Intake Power", intakePower);

    upperP = Preferences.getDouble("upperPID P", upperP);
    upperI = Preferences.getDouble("upperPID I", upperI);
    lowerP = Preferences.getDouble("lowerPID P", lowerP);
    lowerI = Preferences.getDouble("lowerPID I", lowerI);

    //Controller.setP(upperP);
    //Controller.setI(upperI);
    //lowerController.setP(lowerP);
    //lowerController.setI(lowerI);
    //setNeoSpeed(power);
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(7)){
      power = -1500;
      lowerPower = 1500;
      intakePower = 0.7;
    }
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(5)){
      power = -3500;
      lowerPower = 3500;
      intakePower = 0.7;
    }
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(6)){
      power = 0;
      lowerPower = 0;
      intakePower = 0;
    }
     if(RobotContainer.getIntakeJoy().getRawButtonPressed(3)){
      power = -1000;
      lowerPower = 2500;
    }

    if(RobotContainer.getIntakeJoy().getRawButtonPressed(4)){
      power = 60;
      lowerPower = 60;
    }
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(10)){
      power = 1000;
      lowerPower = -1000;
      intakePower = -0.6;
    }
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(12)){
      intakePower += 0.05;
    }
    

    if(RobotContainer.getIntakeXbox().getRawButtonPressed(XboxController.Button.kA.value)){
      power = 0;
      lowerPower = -1000;
    }
    if(RobotContainer.getIntakeXbox().getRawButtonPressed(XboxController.Button.kB.value)){
      power = 4000;
      lowerPower = 1000;
    }
    
    setupperSpeed(power);
    setlowerSpeed(lowerPower);
    setFeedSimple(intakePower);

    
    SmartDashboard.putNumber("upper rpm", (int) getUpperRPM());
    SmartDashboard.putNumber("lower rpm", (int) getLowerRPM());
    SmartDashboard.putNumber("intake RPM", (int) getIntakeRPM());
    SmartDashboard.putBoolean("shooterBeamBreak", getShooterBeam());
    SmartDashboard.putBoolean("intakeBeamBreak", getIntakeBeam());
    // This method will be called once per scheduler run
  }
}