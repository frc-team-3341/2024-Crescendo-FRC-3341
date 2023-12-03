// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingularModule extends SubsystemBase {

  public SwerveModuleIO module;

  /** Creates a new SingularModule. */
  public SingularModule(SwerveModuleIO module) {
    this.module = module;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates module telemetry
    module.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    module.updateSim();
  }
}
