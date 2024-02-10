package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.photonvision.PhotonVision;

public class RobotContainer {


  public RobotContainer() {

    this.configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new TargetAprilTag(new PhotonVision());
  }

  public void initCommandInTeleop() {
  }

  /**
   * Gets Robot.isReal() from RobotContainer (slow when calling every loop)
   *
   * @return If simulated or not
   */

}