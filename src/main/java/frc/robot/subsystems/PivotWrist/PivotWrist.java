//need to bind commands and put constants

package frc.robot.subsystems;

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

// need to add constants: import static frc.robot.Constants.IntakeArm(actually pivot wrist).*; 
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

/**
 * Subsystem to handle the -Originally intake arm- mechanism but this is pivot
 * wrist
 * 
 * - Utilizes one(two?) NEO 550 motor attached to the intake mechanism
 */

public class PivotWrist extends SnailSubsystem {

    private final CANSparkMax pivotWristMotorRight;
    private final CANSparkMax pivotWristMotorLeft;
    private double setpoint;

    private RelativeEncoder primaryEncoder;
    private SparkMaxPIDController wristPID;
    // private DigitalInput limitSwitch;
    private DigitalInput limitSwitch;
    public double speed;
    private double simulationPos = 0;

    private MechanismLigament2d pivotWristMechanism;

    public enum State {
        MANUAL,
        PID;
    }

    State state = State.MANUAL;

    public PivotWrist() {
        // Set motor
        pivotWristMotorRight = new CANSparkMax(PIVOT_WRIST_ID_RIGHT, MotorType.kBrushless);
        pivotWristMotorRight.restoreFactoryDefaults();
        pivotWristMotorRight.setIdleMode(IdleMode.kBrake);
        pivotWristMotorRight.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pivotWristMotorLeft = new CANSparkMax(PIVOT_WRIST_ID_LEFT, MotorType.kBrushless);
        pivotWristMotorLeft.restoreFactoryDefaults();
        pivotWristMotorLeft.setIdleMode(IdleMode.kBrake);
        pivotWristMotorLeft.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pivotWristMotorLeft.follow(pivotWristMotorRight);

        pivotWristMotorRight.setInverted(true);

        // Get Encoder
        primaryEncoder = pivotWristMotorRight.getEncoder();
        primaryEncoder.setPositionConversionFactor(WRIST_ENCODER_PCF); // verify with build
        primaryEncoder.setVelocityConversionFactor(WRIST_ENCODER_PCF / 60);
        resetEncoder();

        // Get PID Controller and set
        wristPID = pivotWristMotorRight.getPIDController();
        wristPID.setP(WRIST_PID[0]);
        wristPID.setI(WRIST_PID[1]);
        wristPID.setD(WRIST_PID[2]);
        wristPID.setFF(WRIST_PID[3]);
        wristPID.setOutputRange(-WRIST_PID_MAX_OUTPUT, WRIST_PID_MAX_OUTPUT);

        limitSwitch = new DigitalInput(WRIST_LIMIT_SWITCH_PORT_ID); // have to get ID later for constant
    }

    /**
     * Update motor outputs according to the current state
     */
    @Override
    public void update() {
        switch (state) {
            case MANUAL:
                pivotWristMotorRight.set(speed);
                break;
            case PID:
                // send the desired setpoint to the PID controller and specify we want to use
                // position control
                wristPID.setReference(setpoint, CANSparkMax.ControlType.kPosition);

                // check our error and update the state if we finish
                if (Math.abs(primaryEncoder.getPosition() - setpoint) < WRIST_PID_TOLERANCE) {
                    endPID();
                }
                break;
        }

        simulationPos += speed * 5;

        if (RobotBase.isSimulation()) {
            // update the mechanism ligament
            pivotWristMechanism.setAngle(simulationPos - 90);
            simulationPos %= 360;
        } else {
            pivotWristMechanism.setAngle(primaryEncoder.getPosition());
        }

        // pivotWristMechanism.setAngle(primaryEncoder.getPosition());

        // if (getlimitSwitch() && state == State.PID) {
        // resetEncoder();
        // } else if (getlimitSwitch() && state == State.MANUAL) {
        // pivotWristMotorRight.set(0);
        // }
    }

    // End PID
    public void endPID() {
        state = State.MANUAL;
        // setpoint = -1257;
    }

    public void resetEncoder() {
        primaryEncoder.setPosition(0.0);
    }

    public void manual(double speed) {
        state = State.MANUAL;
        this.speed = speed;
    }

    // Set PID
    public void setPosition(double setpoint) {
        state = State.PID;
        this.setpoint = setpoint;
    }

    private boolean getlimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void displayShuffleboard() {
        // Display Encoder position and setpoint
        SmartDashboard.putNumberArray("Pivot Wrist Dist PID (pos, setpt)",
                new double[] { primaryEncoder.getPosition(), setpoint });
        SmartDashboard.putString("Pivot Wrist State", state.name());
        SmartDashboard.putNumber("Pivot Wrist Current", pivotWristMotorRight.getOutputCurrent());
        SmartDashboard.putBoolean("Pivot Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Motor Speed", primaryEncoder.getVelocity());
        SmartDashboard.putNumber("Encoder position", primaryEncoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("SimPosWrist", (int)simulationPos);
    }

    @Override
    public void tuningInit() {
        SmartDashboard.putNumber("Pivot Wrist PID P", WRIST_PID[0]);
        SmartDashboard.putNumber("Pivot Wrist PID I", WRIST_PID[1]);
        SmartDashboard.putNumber("Pivot Wrist PID D", WRIST_PID[2]);
        SmartDashboard.putNumber("Pivot Wrist PID FF", WRIST_PID[3]);

        SmartDashboard.putNumber("Pivot Wrist PID Tolerance", WRIST_PID_TOLERANCE);
        SmartDashboard.putNumber("Pivot Wrist PID Max Output", WRIST_PID_MAX_OUTPUT);
        SmartDashboard.putNumber("Pivot Wrist Prof Max Vel", WRIST_MAX_VEL);
        SmartDashboard.putNumber("Pivot Wrist Prof Max Accel", WRIST_MAX_ACC);

        SmartDashboard.putNumber("Pivot Wrist Setpoint Top", WRIST_SETPOINT_TOP);
        SmartDashboard.putNumber("Pivot Wrist Setpoint Bottom", WRIST_SETPOINT_BOT);
    }

    @Override
    public void tuningPeriodic() {
        // Change the P, I, and D values
        WRIST_PID[0] = SmartDashboard.getNumber("Pivot Wrist P", WRIST_PID[0]);
        WRIST_PID[1] = SmartDashboard.getNumber("Pivot Wrist PID I", WRIST_PID[1]);
        WRIST_PID[2] = SmartDashboard.getNumber("Pivot Wrist PID D", WRIST_PID[2]);
        WRIST_PID[3] = SmartDashboard.getNumber("Pivot Wrist PID FF", WRIST_PID[3]);

        WRIST_PID_TOLERANCE = SmartDashboard.getNumber("Pivot Wrist PID Tolerance", WRIST_PID_TOLERANCE);
        WRIST_PID_MAX_OUTPUT = SmartDashboard.getNumber("Pivot Wrist PID Max Output", WRIST_PID_MAX_OUTPUT);

        WRIST_MAX_VEL = SmartDashboard.getNumber("Pivot Wrist Prof Max Vel", WRIST_MAX_VEL);
        WRIST_MAX_ACC = SmartDashboard.getNumber("Pivot Wrist Prof Max Accel", WRIST_MAX_ACC);

        // Set PID
        if (wristPID.getP() != WRIST_PID[0])
            wristPID.setP(WRIST_PID[0]);
        if (wristPID.getI() != WRIST_PID[1])
            wristPID.setI(WRIST_PID[1]);
        if (wristPID.getD() != WRIST_PID[2])
            wristPID.setD(WRIST_PID[2]);
        if (wristPID.getFF() != WRIST_PID[3])
            wristPID.setFF(WRIST_PID[3]);
        if (wristPID.getOutputMin() != -WRIST_PID_MAX_OUTPUT)
            wristPID.setOutputRange(-WRIST_PID_MAX_OUTPUT, WRIST_PID_MAX_OUTPUT);
        if (wristPID.getSmartMotionMaxVelocity(WRIST_PID_SLOT_VEL) != WRIST_MAX_VEL)
            wristPID.setSmartMotionMaxVelocity(WRIST_MAX_VEL, WRIST_PID_SLOT_VEL);
        if (wristPID.getSmartMotionMaxAccel(WRIST_PID_SLOT_ACC) != WRIST_MAX_ACC)
            wristPID.setSmartMotionMaxVelocity(WRIST_MAX_ACC, WRIST_PID_SLOT_ACC);
    }

    /**
     * Returns the state
     */
    public State getState() {
        return state;
    }

    
    public MechanismLigament2d getWristMechanism() {
        return new MechanismLigament2d("Pivot Wrist", WRIST_LENGTH, 0.0, 5, new Color8Bit(Color.kRed));
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        pivotWristMechanism = mechanism;
    }
}