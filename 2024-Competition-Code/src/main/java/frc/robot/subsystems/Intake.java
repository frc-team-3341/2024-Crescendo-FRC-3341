package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  DigitalInput beamBreak1 = new DigitalInput(Constants.BeamBreak.beamID1);
  DigitalInput beamBreak2 = new DigitalInput(Constants.BeamBreak.beamID2);

  RelativeEncoder encoder;

  // Declare a Talon SRX motor controller for the flywheel
  // Initialize the Talon SRX motor controller for the flywheel with a specific
  // port
  private final CANSparkMax intakeRollers = new CANSparkMax(Constants.IntakePorts, MotorType.kBrushless);

  public Intake() {
    // Set the inversion of the flywheel motor controller (whether it runs in the
    // opposite direction)
    intakeRollers.setInverted(false);

    encoder = intakeRollers.getEncoder();

  }

  public boolean getBeambreak1() {
    return !beamBreak1.get();
  }

  public boolean getBeambreak2() {
    return !beamBreak2.get();
  }

  public void setFlywheelPower(double speed) {
    intakeRollers.set(speed);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("beamBreak", getBeambreak1());
    SmartDashboard.putBoolean("beamBreak2", getBeambreak2());
  }

  @Override
  public void simulationPeriodic() {
    // Simulation-specific code
  }

  // Method to set the power of the flywheel motor controller

  // Method to retrieve the current drawn by the flywheel motor controller
  public final double getCurrent() {
    return intakeRollers.getOutputCurrent();
  }

}
