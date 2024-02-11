// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  
  private static Optional<Alliance> alliance = DriverStation.getAlliance();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    SmartDashboard.putBoolean("Is Running", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    SmartDashboard.putBoolean("Is Running", !m_autonomousCommand.isFinished());
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
  
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    alliance = DriverStation.getAlliance();
    
    m_robotContainer.initCommandInTeleop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  
  public static Optional<Alliance> getAlliance() {
    return alliance;
  }
}
