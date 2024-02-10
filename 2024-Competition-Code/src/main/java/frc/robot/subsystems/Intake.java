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

  // Declare a Talon SRX motor controller for the flywheel
    // Initialize the Talon SRX motor controller for the flywheel with a specific port
  private final WPI_TalonSRX FlyWheelTalon = new WPI_TalonSRX(Constants.IntakePorts);

  public Intake(){
      // Set the inversion of the flywheel motor controller (whether it runs in the opposite direction)
  FlyWheelTalon.setInverted(false);
  
  FlyWheelTalon.configFactoryDefault();
  }


 public boolean getSensor(){
  return !beamBreak.get();
 }

  public boolean getSensor2(){
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

      if(!getSensor() && timerStarted){
     timer.stop();
     timerStarted = false;
    }
    


   if(getSensor2()){
    if(timer.get() > 0){
       seconds = timer.get();
      }
      timer.stop();
      timer.reset();
   }
  }

  SmartDashboard.putBoolean("beamBreak", getSensor());
  SmartDashboard.putBoolean("beamBreak2", getSensor2());
  SmartDashboard.putNumber("seconds", seconds);
  SmartDashboard.putNumber("timer", timer.get());
}
         @Override
    public void simulationPeriodic() {
      // Simulation-specific code
    }
  
    // Method to set the power of the flywheel motor controller
   
    // Method to retrieve the current drawn by the flywheel motor controller
    public final double getCurrent() {
      return FlyWheelTalon.getStatorCurrent();
    }

  }
  
