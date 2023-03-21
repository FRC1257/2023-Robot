package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class that all subsystems on our robot should inherit so that the following functions are called at the right times
 */
public abstract class SnailSubsystem extends SubsystemBase {

    /**
     * Runs code every 20 ms
     */
    public abstract void update();

    /**
     * Put data onto SmartDashboard
     */
    public abstract void displayShuffleboard();

    /**
     * Send intial PID and other tuning variables for each subsystem to SmartDashboard
     */
    public abstract void tuningInit();
    
    /**
     * Get data from SmartDashboard
     */
    public abstract void tuningPeriodic();
}
