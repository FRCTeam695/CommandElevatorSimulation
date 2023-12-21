// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements AutoCloseable
{
  // Default Constants/Gains
  private double maxVelFromDash = 2.45;
  private double maxAccelFromDash = 2.45;

  // Hardware interface - only this talks to the outside world
  private final Encoder m_encoder;
  private final PWMSparkMax m_motor;

  // Calculators - poke results in, get results out
  private ProfiledPIDController m_controller;
  private ElevatorFeedforward m_feedforward;

  // because ProfiledPIDController doesn't calculate acceleration
  // and because this elevator has enough mass that acceleration
  // feedforward is worthwhile, and makes a difference
  private double prevGoalVelocity = 0;


  // Tell Glass to create a visualization of the elevator
  private static final double VISUAL_HEIGHT = 10;
  private final Mechanism2d m_elevatorDisplay = new Mechanism2d(20, VISUAL_HEIGHT);
  private final MechanismRoot2d m_elevatorStickStart = m_elevatorDisplay.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorStickDrawing =
      m_elevatorStickStart.append(
          new MechanismLigament2d("Elevator", 0, 90));
  
  
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
    initConstantsInternal();

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_elevatorDisplay);
  }

  // Public interface -- this (and only this) is how you control the elevator
  
  /** Read control loop constants/limits from the dashboard */
  public Command initConstants()
  {
    return runOnce(()->initConstantsInternal());
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command goToHeight(double goal)
  {
    return run(()-> reachGoal(goal));
  }

  public Command manualControl(DoubleSupplier propVBus)
  {
    return run(()->m_motor.set(propVBus.getAsDouble()));
  }

  /** Whether the profile setpoint has reached goal state */
  public boolean atGoal()
  {
    return m_controller.atGoal();
  }

  /** Stop the control loop and motor output. */
  public Command stop()
  {
    return run(()-> internalStop()).ignoringDisable(true);
  }

  // Subsystem Boilerplate
  @Override
  public void periodic()
  {
     // Update the telemetry, including mechanism visualization, regardless of mode.
    updateTelemetry();
  }

  @Override
  public void close()
  {
      m_elevatorDisplay.close();
  }


  // Internal controls ONLY

  private void initConstantsInternal()
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

  private void reachGoal(double goal)
  {
    m_controller.setGoal(goal);

    double velocity = m_controller.getSetpoint().velocity;
    double acceleration = (velocity - prevGoalVelocity) / m_controller.getPeriod();

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(velocity, acceleration);
    
    // having a plot of the PID (well, PD) and feedforward outputs is helpful
    // during tuning; the PD output should be close to zero, and if it predictably
    // deviates, the shape of the deviation indicates which of the
    // feedforward components should be modified
    SmartDashboard.putNumber("PIDOutput", pidOutput);
    SmartDashboard.putNumber("FeedforwardOutput", feedforwardOutput);
    
    m_motor.setVoltage(pidOutput + feedforwardOutput);

    prevGoalVelocity = velocity;
  }

  private void internalStop()
  {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  private void updateTelemetry()
  {
    SmartDashboard.putNumber("setpointPosition",m_controller.getSetpoint().position);
    SmartDashboard.putNumber("setpointVelocity",m_controller.getSetpoint().velocity);
    SmartDashboard.putNumber("ElevatorHeight", m_encoder.getDistance());
    SmartDashboard.putNumber("ElevatorGoalHeight", m_controller.getGoal().position);
    SmartDashboard.putBoolean("ElevatorAtGoal", atGoal());

    m_elevatorStickDrawing.setLength(m_encoder.getDistance() / Constants.kMaxElevatorHeightMeters * VISUAL_HEIGHT);
  }
}
