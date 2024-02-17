// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

enum IntakeState {
  Idle,
  FirstBeamIsBroken,
  SecondBeamIsBroken
}

public class IntakeComm extends Command {
  /** Creates a new IntakeCommand. */

public Intake intake = new Intake();
public double conveyingPower = 0;
private IntakeState internalState = IntakeState.Idle;

  public IntakeComm(Intake intake, double conveyingPower) {
    this.intake = intake;
    this.conveyingPower = conveyingPower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Preferences.initDouble("power", conveyingPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyingPower = Preferences.getDouble("power", conveyingPower);

    switch(internalState) {
      case Idle:
        intake.setFlywheelPower(0.1); // Positive is conveying into/up the chassis
        if (intake.getBeambreak1()) {
          internalState = IntakeState.FirstBeamIsBroken;
        }
    
      case FirstBeamIsBroken:
        intake.setFlywheelPower(conveyingPower);

        if (intake.getBeambreak2()) {
         internalState = IntakeState.SecondBeamIsBroken;
        }

      case SecondBeamIsBroken:
        intake.setFlywheelPower(0.0);
        end(true); // End the program
    }
    
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setFlywheelPower(0); //at the end, the speed stops
  }
}
