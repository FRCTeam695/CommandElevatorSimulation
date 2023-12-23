package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorSimulation
{
    // Our link to the elevator control code, including sim classes that
    // allow us to inject values into it
    // Connection between the controller and the plant
    private final Encoder elevatorEncoder;
    private final PWMMotorController elevatorMotor;
    private final EncoderSim m_encoderSim;
    private final PWMSim m_motorSim;

    // Simulation of the plant
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);
    private final ElevatorSim m_elevatorSim =
        new ElevatorSim(
            m_elevatorGearbox,
            Constants.kElevatorGearing,
            Constants.kCarriageMass,
            Constants.kElevatorDrumRadius,
            Constants.kMinElevatorHeightMeters,
            Constants.kMaxElevatorHeightMeters,
            Constants.kSimulateGravity,
            0,
            VecBuilder.fill(0.005));

    // Simulate the behavior of an elevator, using the supplied connections
    public ElevatorSimulation(Encoder encoder, PWMMotorController motor)
    {
        elevatorEncoder = encoder;
        elevatorMotor = motor;

        m_encoderSim = new EncoderSim(elevatorEncoder);
        m_motorSim = new PWMSim(elevatorMotor);
    }

    /** Advance the simulation. */
    public void elevatorSimulationPeriodic()
    {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
        m_encoderSim.setRate(m_elevatorSim.getVelocityMetersPerSecond());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

}