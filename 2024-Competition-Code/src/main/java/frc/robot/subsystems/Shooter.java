// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private RelativeEncoder relativeEncoder;
  private RelativeEncoder relativeEncoder2;
  public final CANSparkMax upperShooter;
  public final CANSparkMax lowerShooter;
  public double setPoint;
  public double upperRPM = 0;
  public double lowerRPM = 0;
  private double intakePower = 0;

  private double upperP = 0.0001;
  private double upperI = 0.000001;
  private double lowerP = 0.0001;
  private double lowerI = 0.000001;
  private double intakeP = 0.001;
  private double intakeI = 0.000001;

  private double upperFeedforward = 1.0/6100.0;
  private double lowerFeedforward = 1.0/6100.0;
  private double intakeFeedForward = 1.0/6100.0;

  public SparkPIDController upperController;
  public SparkPIDController lowerController;
  public SparkPIDController intakeController;
  
  private RelativeEncoder intakeEncoder;
  public final CANSparkMax intakeMax;
  //private double power = 0;
  DigitalInput shooterBeamBreak;
  DigitalInput intakeBeamBreak;
  
  // Creates a new Shooter.
  public Shooter() {
    shooterBeamBreak = new DigitalInput(Constants.IntakeConstants.shooterBeamBreak);
    intakeBeamBreak = new DigitalInput(Constants.IntakeConstants.intakeBeamBreak);  

    upperShooter = new CANSparkMax(Constants.ShooterConstants.upperShooter, MotorType.kBrushless);
    lowerShooter = new CANSparkMax(Constants.ShooterConstants.lowerShooter, MotorType.kBrushless);
    intakeMax = new CANSparkMax(Constants.IntakeConstants.feederMax, MotorType.kBrushless);

    upperShooter.restoreFactoryDefaults();
    lowerShooter.restoreFactoryDefaults();
    intakeMax.restoreFactoryDefaults();
    upperShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    upperShooter.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);
    lowerShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
    lowerShooter.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);
    relativeEncoder = upperShooter.getEncoder();
    relativeEncoder2 = lowerShooter.getEncoder();
    
    
    upperController = upperShooter.getPIDController();
    upperController.setP(upperP);
    upperController.setI(upperI);
    upperController.setD(0);
    //Controller.setFF(1.0/5891.0); decently working
    upperController.setFF(upperFeedforward);

    lowerController = lowerShooter.getPIDController();
    intakeController = intakeMax.getPIDController();
    lowerController.setP(lowerP);
    lowerController.setI(lowerI);
    lowerController.setD(0);
    lowerController.setFF(lowerFeedforward);
    intakeController.setP(intakeP);
    intakeController.setI(intakeI);
    intakeController.setD(0);
    intakeController.setFF(intakeFeedForward);

    Preferences.initDouble("power", upperRPM);
    Preferences.initDouble("Lower Power", lowerRPM);
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
    
    
    
    
    upperShooter.setInverted(false); // if upper is negative and lower is positive, the note will be shot out. With these settings, a positive power value needs to be supplied
    lowerShooter.setInverted(true); 
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
    upperController.setReference(setPoint, ControlType.kVelocity);
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

  public void setintakeSpeed(double setPoint){
    intakeController.setReference(setPoint, ControlType.kVelocity);
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
  
  public boolean setpointReached(double currentRPM, double setpoint){
    return (Math.abs(currentRPM-setpoint) <= 10);
  }

  public void setUpperRPM(int val){
    this.upperRPM = val;
  }

  public void setLowerRPM(int val){
    this.lowerRPM = val;
  }

  public void setIntakePower(double val){
    this.intakePower = val;
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

    //Stop all motors
    // if(RobotContainer.getIntakeJoy().getRawButtonPressed(2)){ 
    //   upperRPM = 0;
    //   lowerRPM = 0;
    //   intakePower = 0;
    // }

    // Shoots 
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(1)){ //shoot
      intakePower = 1.0;
    }

    // Gets shooter wheels up to speed for speaker (untested)
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(3)){ //prep flywheels for speaker
      upperRPM = 3500;
      lowerRPM = 3500;
    }

    // Intakes note from source (untested)
    /*if(RobotContainer.getIntakeJoy().getRawButtonPressed(6)){ //intake from source
      upperRPM = -1500;
      lowerRPM = -1500;
      intakePower = -0.6;
    }*/

    // Gets shooter wheels up to speed for amp (tested but could be improved)
    // Works when the robot is aligned right in front of amp
    // 120 upper and 80 lower
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(4)){ //prep flywheels for shooting into amp
      upperRPM = 360;
      lowerRPM = 60;
    }

    if(RobotContainer.getIntakeJoy().getRawButtonPressed(7)){ //prep flywheels for shooting into amp
      upperRPM = 80;
      lowerRPM = 0;
    }
    

    /*if(RobotContainer.getIntakeJoy().getRawButtonPressed(3)){
      intakePower = 1.0;
    }*/
/**
    // Increment intake speed
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(7)){
      intakePower += 0.1;
    }
*/
    // Decrement intake speed
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(8)){
      intakePower -= 0.1;
    }

    // Incremenet upper shooter RPM
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(9)){
      upperRPM += 100;
    }

    // Decrement upper shooter RPM
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(10)){
      upperRPM -= 100;
    }

    // Incremenet lower shooter RPM
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(11)){
      lowerRPM += 100;
    }

    // Decrement lower shooter RPM
    if(RobotContainer.getIntakeJoy().getRawButtonPressed(12)){
      lowerRPM -= 100;
    }
    

    /*if(RobotContainer.getIntakeXbox().getRawButtonPressed(XboxController.Button.kA.value)){
      upperRPM = 0;
      lowerRPM = -1000;
    }
    if(RobotContainer.getIntakeXbox().getRawButtonPressed(XboxController.Button.kB.value)){
      upperRPM = 4000;
      lowerRPM = 1000;
    }*/
    
    setupperSpeed(upperRPM);
    setlowerSpeed(lowerRPM);
    //setintakeSpeed(intakePower);
    setFeedSimple(intakePower);
    
    SmartDashboard.putNumber("upper rpm", (int) getUpperRPM());
    SmartDashboard.putNumber("lower rpm", (int) getLowerRPM());
    SmartDashboard.putNumber("intake RPM", (int) getIntakeRPM());

    SmartDashboard.putNumber("Desired upper rpm", this.upperRPM);
    SmartDashboard.putNumber("Desired lower rpm", this.lowerRPM);
    SmartDashboard.putNumber("Desired intake RPM", this.intakePower);

    SmartDashboard.putBoolean("shooterBeamBreak", getShooterBeam());
    SmartDashboard.putBoolean("intakeBeamBreak", getIntakeBeam());
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Upper Setpoint Reached", setpointReached(getUpperRPM(), upperRPM));
    SmartDashboard.putBoolean("Lower Setpoint Reached", setpointReached(getLowerRPM(), lowerRPM));
  }
}