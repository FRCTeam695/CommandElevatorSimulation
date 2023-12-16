package frc.robot;

public interface ElevatorSimulationInterface {

    /** Advance the simulation. */
    void elevatorSimulationPeriodic();

    public static ElevatorSimulationInterface empty()
    {
        return new NullElevatorSimulation();
    }

    public static final class NullElevatorSimulation implements ElevatorSimulationInterface
    {
        private NullElevatorSimulation()
        {}

        @Override
        public void elevatorSimulationPeriodic() {}
    }

}