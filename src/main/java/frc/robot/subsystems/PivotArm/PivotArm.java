package frc.robot.subsystems.PivotArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SnailSubsystem;

import static frc.robot.Constants.PivotArm.*;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

public class PivotArm extends SnailSubsystem {
    private PivotArmIO io;
    private MechanismLigament2d armMechanism;

    public PivotArm(PivotArmIO io) {
        this.io = io;
    }

    @Override
    public void update() {
        io.updateIO();
        armMechanism.setAngle(io.getArmAngle());
    }

    public void setPosition(double setpoint) {
        io.setPosition(setpoint);
    }

    @Override
    public void displayShuffleboard() {
        io.displayShuffleboardIO();
    }

    @Override
    public void tuningInit() {
        io.tuningInitIO();
    }

    @Override
    public void tuningPeriodic() {
        io.tuningPeriodicIO();
    }

    public void manualControl(double newSpeed) {
        io.manualControl(newSpeed);
    }

    public PivotArmIO.State getState() { return io.getState(); }

    public void setMechanism(MechanismLigament2d mechanism) {
        armMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return armMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", ARM_LENGTH, 0, 5, new Color8Bit(Color.kAqua));
    }
}
