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
    private final Encoder elevatorEncoder = new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
    private final PWMSparkMax elevatorMotor = new PWMSparkMax(Constants.kMotorPort);

    private final Elevator m_elevator = new Elevator(elevatorEncoder, elevatorMotor);

    private ElevatorSimulationInterface m_elevatorSimulation;

    public RobotContainer()
    {
        if(RobotBase.isReal())
            m_elevatorSimulation = new NullElevatorSimulation();
        else
            m_elevatorSimulation = new ElevatorSimulation(elevatorEncoder, elevatorMotor);

        
        // This just makes sure that our simulation code knows that the motor's off.
        CommandScheduler.getInstance().setDefaultCommand(m_elevator, m_elevator.stop());
        
        

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
    }

    public void teleopInit()
    {
        m_elevator.initConstants();
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
