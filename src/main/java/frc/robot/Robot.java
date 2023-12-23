// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot
{
  private RobotContainer m_robotContainer;
 

  @Override
  public void robotInit()
  {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage());
  }

  @Override
  public void teleopInit()
  {
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_robotContainer.simulationPeriodic();
  }

  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void close() {
    m_robotContainer.close();
    super.close();
  }
}
