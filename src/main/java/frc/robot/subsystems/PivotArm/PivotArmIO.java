package frc.robot.subsystems.PivotArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.PivotArm.*;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public interface PivotArmIO {
    public enum State {
        MANUAL,
        PID
    }

    public default void setPosition(double setpoint) {}

    public default void manualControl(double newSpeed) {}

    public default void updateIO() {}

    public default void displayShuffleboardIO() {}

    public default void tuningInitIO() {}

    public default void tuningPeriodicIO() {}

    public default State getState() { return State.MANUAL; }

    public default double getArmAngle() { return 0; }

}
