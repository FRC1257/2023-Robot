package frc.robot.subsystems;

// subsystem base class
import frc.robot.subsystem.SnailSubsystem;

/*
    ToCharge Subsystem
    Moves robot to and engages with the charging station.
    Uses points on either side of the station along with points to traverse obstacles
    and one point on the station itself to move to and engage with the charging
    station and earn a bonus.
*/
public abstract class ToChargeSubsystem extends SnailSubsystem {
    public void update() {
        // updateTrajectory
    }
  
    public void displayShuffleboard() {
        // send distance data back to the drive team
    }
  
    public void tuningInit() {
        // idk what this is lmao
    }
  
    public void tuningPeriodic() {
        // same as before ^^^^
    }
}
