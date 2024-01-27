package frc.robot.subsystems.intake;

public class BeamBreak {
    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class BeamBreak extends SubsystemBase {
 /** Creates a new beamBreak. */


 boolean timerStarted = false;
 Timer timer = new Timer();
 double seconds;
 DigitalInput beamBreak = new DigitalInput(Constants.BeamBreak.beamID1);
 //DigitalInput beamBreak2 = new DigitalInput(Constants.BeamBreak.beamID2);
 // DigitalInput beamBreak2 = new DigitalInput(Constants.BeamBreak.beamID2);


 public BeamBreak() {
 }


 public boolean getSensor(){
   return !beamBreak.get();
 }
 //public boolean getSecondSensor(){
 //return !beamBreak2.get();
 //}


 @Override
 public void periodic() {
   // This method will be called once per scheduler run
   if(getSensor() && !timerStarted){
     timer.reset();
     timer.start();
     timerStarted = true;
   }
  
   if(!getSensor() && timerStarted){
     timer.stop();
     timerStarted = false;
   }


   // if(getSecondSensor()){
   //   if(timer.get() > 0){
   //     seconds = timer.get();
   //   }
     // timer.stop();
     // timer.reset();


   SmartDashboard.putBoolean("beamBreak1", getSensor());
   //SmartDashboard.putBoolean("beamBreak2", getSecondSensor());
   SmartDashboard.putNumber("seconds", seconds);
   SmartDashboard.putNumber("timer", timer.get());


 }
}

