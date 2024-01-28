// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;



import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private RelativeEncoder relativeEncoder;
  private RelativeEncoder relativeEncoder2;
  private final CANSparkMax UpperShooter = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax LowerShooter = new CANSparkMax(2, MotorType.kBrushless);
  public double TicksToRPM = 4096/60/100;
  public BangBangController bang = new BangBangController();
  public double setPoint;
  private SimpleMotorFeedforward feedF = new SimpleMotorFeedforward(feedForwardConsts.kS, feedForwardConsts.kV, feedForwardConsts.kA);

  public Shooter() {
    UpperShooter.restoreFactoryDefaults(); //configs it to default when taken out of factory
    LowerShooter.restoreFactoryDefaults();

    relativeEncoder = UpperShooter.getEncoder();
    relativeEncoder2 = LowerShooter.getEncoder();






    UpperShooter.setInverted(true);
    LowerShooter.setInverted(true);
  }


  public double getUpperRPM(){
    return relativeEncoder.getVelocity()/TicksToRPM;
  }
  public double getLowerRPM(){
    return ((RelativeEncoder) LowerShooter).getVelocity()/TicksToRPM;
  }
  public void resetLowerShooter(){
    relativeEncoder.setPosition(0);
  }
  public void resetUpperShooter(){
    relativeEncoder2.setPosition(0);
  }

    public void setUpperSpeed(double setPoint){
      if(setPoint == 0){
        UpperShooter.set(setPoint);
      }
        else{
          UpperShooter.set(bang.calculate(getLowerRPM(),setPoint));
        }
      
    }

    public void setLowerSpeed(double setPoint){
      if(setPoint == 0){
        LowerShooter.set(setPoint);
      }
        else{
          LowerShooter.set(bang.calculate(getLowerRPM(),setPoint));
        }
      
    }
  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }
}

