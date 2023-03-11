//need to bind commands and put constants

package frc.robot.subsystems.PivotWrist;

import static frc.robot.Constants.PivotWrist.*;
import static frc.robot.Constants.ElectricalLayout.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SnailSubsystem;

// need to add constants: import static frc.robot.Constants.IntakeArm(actually pivot wrist).*; 
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

/**
 * Subsystem to handle the -Originally intake arm- mechanism but this is pivot
 * wrist
 * 
 * - Utilizes one(two?) NEO 550 motor attached to the intake mechanism
 */

public class PivotWrist extends SnailSubsystem {
    private PivotWristIO io;
    private MechanismLigament2d wristMechanism;

    public PivotWrist(PivotWristIO io) {
        this.io = io;
    }

    @Override
    public void update() {
        io.updateIO();
        wristMechanism.setAngle(io.getWristAngle());
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

    public PivotWristIO.State getState() { return io.getState(); }

    public void setMechanism(MechanismLigament2d mechanism) {
        wristMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return wristMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", WRIST_LENGTH, 0, 5, new Color8Bit(Color.kAqua));
    }

    public MechanismLigament2d getWristMechanism() {
        return new MechanismLigament2d("Pivot Wrist", WRIST_LENGTH, 0.0, 5, new Color8Bit(Color.kRed));
    }
}

/*
 *  public enum State {
        MANUAL,
        PID;
    }

    State state = State.MANUAL;

 

    /**
     * Update motor outputs according to the current state
     */
 
  

     

    //  if (RobotBase.isSimulation()) {
    //     // update the mechanism ligament
    //     pivotWristMechanism.setAngle(simulationPos - 90);
    //     simulationPos %= 360;
    // } else {
    //     pivotWristMechanism.setAngle(primaryEncoder.getPosition());
    // }

    // pivotWristMechanism.setAngle(primaryEncoder.getPosition());

    // if (getlimitSwitch() && state == State.PID) {
    // resetEncoder();
    // } else if (getlimitSwitch() && state == State.MANUAL) {
    // pivotWristMotorRight.set(0);
    // }