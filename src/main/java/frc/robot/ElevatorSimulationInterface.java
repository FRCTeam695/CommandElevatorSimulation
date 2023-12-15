package frc.robot;

public interface ElevatorSimulationInterface extends AutoCloseable {

    /** Advance the simulation. */
    void elevatorSimulationPeriodic();

}