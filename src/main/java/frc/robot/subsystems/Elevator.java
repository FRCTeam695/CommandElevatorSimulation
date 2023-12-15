// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase
{
  // Default Constants/Gains
  private double maxVelFromDash = 2.45;
  private double maxAccelFromDash = 2.45;

  // Hardware interface
  private final Encoder m_encoder;
  private final PWMSparkMax m_motor;

  // Calculators/Solvers
  private ProfiledPIDController m_controller;
  private ElevatorFeedforward m_feedforward;
  
  
  public Elevator(Encoder encoder, PWMSparkMax motor)
  {
    m_encoder = encoder;
    m_motor = motor;

    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);

    // I don't like that this is currently in two sections, but I haven't figured out
    // what a better way would be -- it probably should use Preferences, and it would
    // also make for a good example if the gains and feedforward constants were included
    SmartDashboard.putNumber("maxVelocity", maxVelFromDash);
    SmartDashboard.putNumber("maxAcceleration", maxAccelFromDash);
    initConstants();
  }

  public void initConstants()
  {
    maxVelFromDash = SmartDashboard.getNumber("maxVelocity", maxVelFromDash);
    maxAccelFromDash = SmartDashboard.getNumber("maxAcceleration", maxAccelFromDash);

    m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(maxVelFromDash, maxAccelFromDash));
    
    m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  private void reachGoal(double goal)
  {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  public Command goToHeight(double goal)
  {
    return run(()-> reachGoal(goal));
  }

  public Command stop()
  {
    return run(()-> internalStop()).ignoringDisable(true);
  }

  /** Stop the control loop and motor output. */
  private void internalStop()
  {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  @Override
  public void periodic()
  {
     // Update the telemetry, including mechanism visualization, regardless of mode.
    updateTelemetry();
  }

  private void updateTelemetry()
  {
    SmartDashboard.putNumber("setpointPosition",m_controller.getSetpoint().position);
    SmartDashboard.putNumber("setpointVelocity",m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("ElevatorHeight", m_encoder.getDistance());
    SmartDashboard.putNumber("ElevatorGoalHeight", m_controller.getGoal().position);
  }

  
}
