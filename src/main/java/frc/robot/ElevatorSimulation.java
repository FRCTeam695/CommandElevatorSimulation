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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSimulation implements ElevatorSimulationInterface
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
            true,
            0,
            VecBuilder.fill(0.01));

    // Tell Glass to create a visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(20, 10);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));


    public ElevatorSimulation(Encoder encoder, PWMMotorController motor)
    {
        elevatorEncoder = encoder;
        elevatorMotor = motor;

        m_encoderSim = new EncoderSim(elevatorEncoder);
        m_motorSim = new PWMSim(elevatorMotor);

        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    /** Advance the simulation. */
    @Override
    public void elevatorSimulationPeriodic()
    {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

        // Update elevator visualization from position
        m_elevatorMech2d.setLength(elevatorEncoder.getDistance() / Constants.kMaxElevatorHeightMeters * 10);
    }

    @Override
    public void close()
    {
        m_mech2d.close();
    }

}