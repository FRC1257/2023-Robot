//need to bind commands and put constants

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.PivotWristConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SnailSubsystem;

import static frc.robot.Constants.ElectricalLayout.*;
// need to add constants: import static frc.robot.Constants.IntakeArm(actually pivot wrist).*; 
import static frc.robot.Constants.NEO_CURRENT_LIMIT;

/**
 * Subsystem to handle the -Originally intake arm- mechanism but this is pivot wrist
 * 
 * - Utilizes one(two?) NEO 550 motor attached to the intake mechanism
 */

public class PivotWrist extends SnailSubsystem {

    private final CANSparkMax pivotWristMotor;
    private final CANSparkMax pivotWristMotor2;
    private double setpoint;

    private RelativeEncoder primaryEncoder;
    private SparkMaxPIDController wristPID;
    private boolean isPIDFinished;
    // private DigitalInput limitSwitch;
    DigitalInput limitSwitch;
    public double PIVOT_WRIST_RAISE_SPEED;
    public double PIVOT_WRIST_NEUTRAL_SPEED;
    public double PIVOT_WRIST_LOWER_SPEED;

    public enum State {
        MANUAL,
        PID;
    }
    State state = State.MANUAL;

    public PivotWrist() {
        // Set motor
        pivotWristMotor = new CANSparkMax(Constants.PIVOT_WRIST_ID1, MotorType.kBrushless);
        pivotWristMotor.restoreFactoryDefaults();
        pivotWristMotor.setIdleMode(IdleMode.kBrake);
        pivotWristMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pivotWristMotor2 = new CANSparkMax(Constants.PIVOT_WRIST_ID2, MotorType.kBrushless);
        pivotWristMotor2.restoreFactoryDefaults();
        pivotWristMotor2.setIdleMode(IdleMode.kBrake);
        pivotWristMotor2.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        pivotWristMotor2.follow(pivotWristMotor);

        pivotWristMotor.setInverted(true);

        // Get Encoder
        primaryEncoder = pivotWristMotor.getEncoder();
        primaryEncoder.setPositionConversionFactor(PivotWristConstants.wristEnconderPCF); // verify with build
        primaryEncoder.setVelocityConversionFactor(PivotWristConstants.wristEnconderPCF / 60);
        resetEncoder();

        // Get PID Controller and set
        wristPID = pivotWristMotor.getPIDController();
        wristPID.setP(PivotWristConstants.wristPID[0]);
        wristPID.setI(PivotWristConstants.wristPID[1]);
        wristPID.setD(PivotWristConstants.wristPID[2]);
        wristPID.setFF(PivotWristConstants.wristPID[3]);
        wristPID.setOutputRange(-PivotWristConstants.wrist_PID_MAX_OUTPUT, PivotWristConstants.wrist_PID_MAX_OUTPUT);

        limitSwitch = new DigitalInput(PivotWristConstants.wrist_LIMIT_SWITCH_PORT_ID); //have to get ID later for constant
        isPIDFinished = false;
    }
    
    /**
     * Update motor outputs according to the current state
     */
    @Override
    public void update() {
        switch(state) {
            case MANUAL:
            // put each speed case in constants so something like intakeArmMotor.set(INTAKE_ARM_LOWER_SPEED);
                PIVOT_WRIST_RAISE_SPEED = 1257;
                PIVOT_WRIST_NEUTRAL_SPEED = 0.0;
                PIVOT_WRIST_LOWER_SPEED = -1257;
                break;
            case PID:
                // send the desired setpoint to the PID controller and specify we want to use position control
                wristPID.setReference(setpoint, CANSparkMax.ControlType.kPosition);

                // check our error and update the state if we finish
                if(Math.abs(primaryEncoder.getPosition() - setpoint) < PivotWristConstants.wrist_PID_TOLERANCE) {
                    endPID();
                }
                break;
        }

        // if (getlimitSwitch() && state == State.PID) {
        //     resetEncoder();
        // }
    }
    
    // End PID
    public void endPID() {
        state = State.MANUAL;
        //setpoint = -1257;
    }

    public void resetEncoder() {
        primaryEncoder.setPosition(0.0);
    }

    public void manual() {
        state = State.MANUAL;
        
    }

    public void raise() {
        if (state.equals(State.MANUAL)) {
            pivotWristMotor.set(PIVOT_WRIST_RAISE_SPEED);
        }
    }

    public void lower() {
        if (state.equals(State.MANUAL)) {
            pivotWristMotor.set(PIVOT_WRIST_LOWER_SPEED);
        }
    }

     public void neutral() {
        if (state.equals(State.MANUAL)) {
            pivotWristMotor.set(PIVOT_WRIST_NEUTRAL_SPEED);
        }
    }

    // Set PID
    public void setPosition(double setpoint) {
        state = State.PID;
        if (!isPIDFinished) {
            this.setpoint = setpoint;
        }
    }

    private boolean getlimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void displayShuffleboard() {
        // Display Encoder position and setpoint
        SmartDashboard.putNumberArray("Pivot Wrist Dist PID (pos, setpt)", new double[] {primaryEncoder.getPosition(), setpoint});
        SmartDashboard.putString("Pivot Wrist State", state.name());
        SmartDashboard.putNumber("Pivot Wrist Current", pivotWristMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Pivot Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Motor Speed", primaryEncoder.getVelocity());
        SmartDashboard.putNumber("Encoder position", primaryEncoder.getPosition());
        SmartDashboard.putNumber("Setpoint", setpoint);
    }
    
    @Override
    public void tuningInit() {
        SmartDashboard.putNumber("Pivot Wrist PID P", INTAKE_ARM_PID[0]);
        SmartDashboard.putNumber("Pivot Wrist PID I", INTAKE_ARM_PID[1]);
        SmartDashboard.putNumber("Pivot Wrist PID D", INTAKE_ARM_PID[2]);
        SmartDashboard.putNumber("Pivot Wrist PID FF", INTAKE_ARM_PID[3]);

        SmartDashboard.putNumber("Pivot Wrist PID Tolerance", INTAKE_ARM_PID_TOLERANCE);
        SmartDashboard.putNumber("Pivot Wrist PID Max Output", INTAKE_ARM_PID_MAX_OUTPUT);
        SmartDashboard.putNumber("Pivot Wrist Prof Max Vel", INTAKE_ARM_PROFILE_MAX_VEL);
        SmartDashboard.putNumber("Pivot Wrist Prof Max Accel", INTAKE_ARM_PROFILE_MAX_ACC);
        
        SmartDashboard.putNumber("Pivot Wrist Setpoint Top", INTAKE_SETPOINT_TOP);
        SmartDashboard.putNumber("Pivot Wrist Setpoint Bottom", INTAKE_SETPOINT_BOT);
    }

    @Override
    public void tuningPeriodic() {
        // Change the P, I, and D values
        INTAKE_ARM_PID[0] = SmartDashboard.getNumber("Pivot Wrist P", INTAKE_ARM_PID[0]);
        INTAKE_ARM_PID[1] = SmartDashboard.getNumber("Pivot Wrist PID I", INTAKE_ARM_PID[1]);
        INTAKE_ARM_PID[2] = SmartDashboard.getNumber("Pivot Wrist PID D", INTAKE_ARM_PID[2]);
        INTAKE_ARM_PID[3] = SmartDashboard.getNumber("Pivot Wrist PID FF", INTAKE_ARM_PID[3]);
        
        INTAKE_ARM_PID_TOLERANCE = SmartDashboard.getNumber("Pivot Wrist PID Tolerance", INTAKE_ARM_PID_TOLERANCE);
        INTAKE_ARM_PID_MAX_OUTPUT = SmartDashboard.getNumber("Pivot Wrist PID Max Output", INTAKE_ARM_PID_MAX_OUTPUT);
        
        INTAKE_ARM_PROFILE_MAX_VEL = SmartDashboard.getNumber("Pivot Wrist Prof Max Vel", INTAKE_ARM_PROFILE_MAX_VEL);
        INTAKE_ARM_PROFILE_MAX_ACC = SmartDashboard.getNumber("Pivot Wrist Prof Max Accel", INTAKE_ARM_PROFILE_MAX_ACC);
        
        INTAKE_SETPOINT_TOP = SmartDashboard.getNumber("Intake Setpoint Top", INTAKE_SETPOINT_TOP);
        INTAKE_SETPOINT_BOT = SmartDashboard.getNumber("Intake Setpoint Bottom", INTAKE_SETPOINT_BOT);
        
        // Set PID
        if(wristPID.getP() != INTAKE_ARM_PID[0])  wristPID.setP(INTAKE_ARM_PID[0]);
        if(wristPID.getI() != INTAKE_ARM_PID[1]) wristPID.setI(INTAKE_ARM_PID[1]);
        if(wristPID.getD() != INTAKE_ARM_PID[2]) wristPID.setD(INTAKE_ARM_PID[2]);
        if(wristPID.getFF() != INTAKE_ARM_PID[3]) wristPID.setFF(INTAKE_ARM_PID[3]);
        if(wristPID.getOutputMin() != -INTAKE_ARM_PID_MAX_OUTPUT) wristPID.setOutputRange(-INTAKE_ARM_PID_MAX_OUTPUT, INTAKE_ARM_PID_MAX_OUTPUT);
        if(wristPID.getSmartMotionMaxVelocity(INTAKE_ARM_PID_SLOT_VEL) != INTAKE_ARM_PROFILE_MAX_VEL) wristPID.setSmartMotionMaxVelocity(INTAKE_ARM_PROFILE_MAX_VEL, INTAKE_ARM_PID_SLOT_VEL);
        if(wristPID.getSmartMotionMaxAccel(INTAKE_ARM_PID_SLOT_ACC) != INTAKE_ARM_PROFILE_MAX_ACC) wristPID.setSmartMotionMaxVelocity(INTAKE_ARM_PROFILE_MAX_ACC, INTAKE_ARM_PID_SLOT_ACC);
    }

    /**
    * Returns the state
    */
    public State getState() {
        return state;
    }

    
}