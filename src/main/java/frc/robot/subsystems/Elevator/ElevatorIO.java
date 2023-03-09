package frc.robot.subsystems.Elevator;

public interface ElevatorIO {
    public enum State {
        MANUAL,
        PID
    }

    public default void setPosition(double setpoint) {}

    public default void manual(double newSpeed) {}

    public default void updateIO() {}

    public default void displayShuffleboardIO() {}

    public default void tuningInitIO() {}

    public default void tuningPeriodicIO() {}

    public default State getState() { return State.MANUAL; }

    public default double getPosition() { return 0; }

}
