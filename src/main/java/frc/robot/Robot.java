// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot{
  private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);

  private final Encoder elevatorEncoder = new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax elevatorMotor = new PWMSparkMax(Constants.kMotorPort);

  private final Elevator m_elevator = new Elevator(elevatorEncoder, elevatorMotor);

  private ElevatorSimulationInterface m_elevatorSimulation;
  private double setPoint = 0;


 

  @Override
  public void robotInit()
  {
    if(RobotBase.isReal())
    {
      m_elevatorSimulation = new NullElevatorSimulation();
    }
    else
    {
      m_elevatorSimulation = new ElevatorSimulation(elevatorEncoder, elevatorMotor);
    }
  }

  @Override
  public void robotPeriodic() {
    // Update the telemetry, including mechanism visualization, regardless of mode.
    m_elevator.updateTelemetry();
  }

  @Override
  public void teleopInit()
  {
    m_elevator.initConstants();
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevatorSimulation.elevatorSimulationPeriodic();
  }

  @Override
  public void teleopPeriodic()
  {
    if (m_joystick.getRawButtonPressed(1))
      setPoint = 0.25;
    else if(m_joystick.getRawButtonPressed(2))
      setPoint = 0.50;
    else if(m_joystick.getRawButtonPressed(3))
      setPoint = 0.75;
    else if(m_joystick.getRawButtonPressed(4))
      setPoint = 1.00;
    else if(m_joystick.getRawButtonPressed(10))
      setPoint = 0.00;
    else
      setPoint = SmartDashboard.getNumber("Goal", setPoint);


    SmartDashboard.putNumber("Goal", setPoint);

    m_elevator.reachGoal(setPoint);
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  @Override
  public void close() {
    elevatorEncoder.close();
    elevatorMotor.close();
    super.close();
  }
}
