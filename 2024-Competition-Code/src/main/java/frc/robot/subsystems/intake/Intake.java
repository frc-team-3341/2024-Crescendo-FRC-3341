package frc.robot.subsystems.intake;

public class Intake {
    
}

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  
    // Declare a Talon SRX motor controller for the flywheel
    private final WPI_TalonSRX FlyWheelTalon;
    


    public Intake() {
      // Initialize the Talon SRX motor controller for the flywheel with a specific port
      FlyWheelTalon = new WPI_TalonSRX(Constants.IntakePorts);

      // Set the inversion of the flywheel motor controller (whether it runs in the opposite direction)
      FlyWheelTalon.setInverted(false);
  
      FlyWheelTalon.configFactoryDefault();
    }
  
    // Method to set the power of the flywheel motor controller
    public void setFlywheelPower(double speed) {
      FlyWheelTalon.set(ControlMode.PercentOutput, speed);
    }
  
    // Method to retrieve the current drawn by the flywheel motor controller
    public double getCurrent() {
      return FlyWheelTalon.getStatorCurrent();
    }
  
    // Override the periodic method for running code during the robot program execution
    @Override
    public void periodic() {
      
    }
    @Override
    public void simulationPeriodic() {
      // Simulation-specific code
    }
  }
  
