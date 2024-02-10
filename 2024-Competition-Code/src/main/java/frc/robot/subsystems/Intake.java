package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  boolean timerStarted = false;
  Timer timer = new Timer();
  double seconds;
  DigitalInput beamBreak1 = new DigitalInput(Constants.BeamBreak.beamID1);
  DigitalInput beamBreak2 = new DigitalInput(Constants.BeamBreak.beamID2);

  RelativeEncoder encoder;

  // Declare a Talon SRX motor controller for the flywheel
  // Initialize the Talon SRX motor controller for the flywheel with a specific
  // port
  private final CANSparkMax flyWheel = new CANSparkMax(Constants.IntakePorts, MotorType.kBrushless);

  public Intake() {
    // Set the inversion of the flywheel motor controller (whether it runs in the
    // opposite direction)
    flyWheel.setInverted(false);

    encoder = flyWheel.getEncoder();

  }

  public boolean getBeambreak1() {
    return !beamBreak1.get();
  }

  public boolean getBeambreak2() {
    return !beamBreak2.get();
  }

  public void setFlywheelPower(double speed) {
    flyWheel.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getBeambreak1() && !timerStarted) {
      timer.reset();
      timer.start();
      timerStarted = true;

      if (!getBeambreak1() && timerStarted) {
        timer.stop();
        timerStarted = false;
      }

      if (getBeambreak2()) {
        if (timer.get() > 0) {
          seconds = timer.get();
        }
        timer.stop();
        timer.reset();
      }
    }

    SmartDashboard.putBoolean("beamBreak", getBeambreak1());
    SmartDashboard.putBoolean("beamBreak2", getBeambreak2());
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
    return flyWheel.getOutputCurrent();
  }

}
