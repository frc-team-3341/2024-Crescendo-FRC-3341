package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
boolean timerStarted = false;
 Timer timer = new Timer();
 double seconds;
 DigitalInput beamBreak = new DigitalInput(Constants.BeamBreak.beamID1);
 DigitalInput beamBreak2 = new DigitalInput(Constants.BeamBreak.beamID2);
   SmartDashboard.putBoolean("beamBreak", getSensor());
      SmartDashboard.putBoolean("beamBreak2", getSensor());
   SmartDashboard.putNumber("seconds", seconds);
   SmartDashboard.putNumber("timer", timer.get());
    // Declare a Talon SRX motor controller for the flywheel
    private final WPI_TalonSRX FlyWheelTalon;
      // Initialize the Talon SRX motor controller for the flywheel with a specific port
      FlyWheelTalon = new WPI_TalonSRX(Constants.IntakePorts);

      // Set the inversion of the flywheel motor controller (whether it runs in the opposite direction)
      FlyWheelTalon.setInverted(false);
  
      FlyWheelTalon.configFactoryDefault();
    }
 public boolean getSensor(){
   return !beamBreak.get();
   return !beamBreak2.get();
 }

 public void setFlywheelPower(double speed) {
      FlyWheelTalon.set(ControlMode.PercentOutput, speed);
    }
  
 @Override
 public void periodic() {
   // This method will be called once per scheduler run
   if(getSensor() && !timerStarted){
     timer.reset();
     timer.start();
     timerStarted = true; 
   }
         @Override
    public void simulationPeriodic() {
      // Simulation-specific code
    }
   }
  
   if(!getSensor() && timerStarted){
     timer.stop();
     timerStarted = false;
   }


   if(getSecondSensor()){
    if(timer.get() > 0){
       seconds = timer.get();
      }
      timer.stop();
      timer.reset();
    //
    //
  
    // Method to set the power of the flywheel motor controller
   
    // Method to retrieve the current drawn by the flywheel motor controller
    public final double getCurrent() {
      return FlyWheelTalon.getStatorCurrent();
    }

  }
  
