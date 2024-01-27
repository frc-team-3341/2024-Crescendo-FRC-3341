// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public final WPI_TalonSRX feedwheel = new WPI_TalonSRX(Constants.IntakeConstants.feeder);
  public double ticks2RPS = 4096/10;
  //private double power = 0;
  DigitalInput beamBreak1 = new DigitalInput(Constants.IntakeConstants.beamBreak1);
  DigitalInput beamBreak2 = new DigitalInput(Constants.IntakeConstants.beamBreak2);
  public Intake() {
    feedwheel.configFactoryDefault();
    resetEncoders();
    feedwheel.setNeutralMode(NeutralMode.Coast);
    feedwheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //Preferences.initDouble("power", power);
  }
  public double getRPS(){
    return feedwheel.getSelectedSensorVelocity()/ticks2RPS*-1;
  }
  public void resetEncoders(){
    feedwheel.setSelectedSensorPosition(0, 0, 10);
  }
  public void setSpeedSimple(double setPoint){
    feedwheel.set(ControlMode.PercentOutput, setPoint);
  }

  public boolean getSensor(){
    return !beamBreak1.get();
  }
   public boolean getSecondSensor(){
     return !beamBreak2.get();
  }
  @Override
  public void periodic() {
    //power = Preferences.getDouble("power", power);
    SmartDashboard.putNumber("RPS", getRPS());
    //setSpeedSimple(power);
    SmartDashboard.putBoolean("beamBreak1", getSensor());
    SmartDashboard.putBoolean("beamBreak2", getSecondSensor());
    
    // This method will be called once per scheduler run
  }
}
