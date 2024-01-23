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
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private RelativeEncoder relativeEncoder;
  private RelativeEncoder relativeEncoder2;
  public final CANSparkMax upperShooter = new CANSparkMax(Constants.Shooter.upperShooter, MotorType.kBrushless);
  public final CANSparkMax lowerShooter = new CANSparkMax(Constants.Shooter.lowerShooter, MotorType.kBrushless);
  BangBangController controller = new BangBangController();
  public double TicksToRPM = 4096/60/100;
  public double setPoint;
  private SimpleMotorFeedforward neoMotorFeedforward = new SimpleMotorFeedforward(Constants.feedForwardConstants.kS, Constants.feedForwardConstants.kV, Constants.feedForwardConstants.kA);
  /** Creates a new Shooter. */
  public Shooter() {
    upperShooter.restoreFactoryDefaults();
    lowerShooter.restoreFactoryDefaults();
    
    relativeEncoder = upperShooter.getEncoder();
    relativeEncoder2 = lowerShooter.getEncoder();
    
    
    
    
    
    
    upperShooter.setInverted(true);
    lowerShooter.setInverted(true);
  }

  

  public double getUpperRPM(){
    return relativeEncoder.getVelocity()/TicksToRPM;
  }
  public double getLowerRPM(){
    return ((RelativeEncoder) lowerShooter).getVelocity()/TicksToRPM;
  }

  public void resetlowerShooter(){
    relativeEncoder.setPosition(0);
  }
   public void resetupperShooter(){
    relativeEncoder2.setPosition(0);
  }

  public void setupperSpeed(double setPoint){
    if (setPoint == 0){
      upperShooter.set(setPoint);
    }
    else{ 
      upperShooter.set(controller.calculate(getUpperRPM(),setPoint));
    }
  }
  public void setlowerSpeed(double setPoint){
    if (setPoint == 0){
      lowerShooter.set(setPoint);
    }
    else{ 
      lowerShooter.set(controller.calculate(getLowerRPM(),setPoint));
    }
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
