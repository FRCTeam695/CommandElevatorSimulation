package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

public class RobotContainer implements AutoCloseable
{

    private final CommandGenericHID m_joystick = new CommandGenericHID(Constants.kJoystickPort);

    // can these go into a hardware abstraction layer?
    // they could go directly into Elevator instead, but this is a bonus if there are multiple
    // robots to deploy code to, and if their motor controllers are ID'd differently, though it isn't
    // helpful if different robots have different types of motor controllers
    private final Encoder elevatorEncoder = new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
    private final PWMSparkMax elevatorMotor = new PWMSparkMax(Constants.kMotorPort);

    private final Elevator m_elevator = new Elevator(elevatorEncoder, elevatorMotor);

    private ElevatorSimulation m_elevatorSimulation;

    public RobotContainer()
    {
        // simulation-only
        if(!RobotBase.isReal())
           m_elevatorSimulation = new ElevatorSimulation(elevatorEncoder, elevatorMotor);

        
        // This just makes sure that our simulation code knows that the motor's off.
        CommandScheduler.getInstance().setDefaultCommand(m_elevator, m_elevator.depower());

        configureBindings();
    }
    
    private void configureBindings()
    {
        new Trigger(()->{return RobotState.isEnabled() && RobotState.isTeleop();}).onTrue(m_elevator.initConstants());

        m_joystick.button(1).onTrue(m_elevator.goToHeight(0.25));
        m_joystick.button(2).onTrue(m_elevator.goToHeight(0.50));
        m_joystick.button(3).onTrue(m_elevator.goToHeight(0.75));
        m_joystick.button(4).onTrue(m_elevator.goToHeight(1.00));
        m_joystick.button(10).onTrue(m_elevator.goToHeight(0.00));
        
        m_joystick.button(5).whileTrue(m_elevator.manualControl(()->-0.2*m_joystick.getRawAxis(3)));
        m_joystick.button(6).whileTrue(m_elevator.closedLoopManualSetpoint(()->-1.25*m_joystick.getRawAxis(3)));
    }

    public void simulationPeriodic()
    {
        m_elevatorSimulation.elevatorSimulationPeriodic();
    }

    public void close() 
    {
        elevatorEncoder.close();
        elevatorMotor.close();
        m_elevator.close();
    }

}
