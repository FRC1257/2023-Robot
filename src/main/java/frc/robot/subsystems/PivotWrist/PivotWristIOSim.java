package frc.robot.subsystems.PivotWrist;

import static frc.robot.Constants.PivotWrist.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotWristIOSim implements PivotWristIO{
    public static class Constants {
        public static final int kMotorPort = 0;
        public static final int kEncoderAChannel = 5; //these channels used to be used by drivetrain
        public static final int kEncoderBChannel = 6;
      
        // The P gain for the PID controller that drives this arm.
        public static final double kDefaultArmKp = 50.0;
        public static final double kDefaultArmSetpointDegrees = 75.0;
      
        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
      
        public static final double kArmReduction = 200;
        public static final double kArmMass = 8.0; // Kilograms
        public static final double kArmLength = WRIST_LENGTH; // Meters
        public static final double kMinAngleRads = Units.degreesToRadians(-180);
        public static final double kMaxAngleRads = Units.degreesToRadians(120);
    }

    private double m_armKp = Constants.kDefaultArmKp;
    private double setpoint = Constants.kDefaultArmSetpointDegrees;
    private double speed;
  
    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  
    // Standard classes for controlling our arm
    private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
    private final Encoder m_encoder = new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
    private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

    private PivotWristIO.State state = PivotWristIO.State.MANUAL;
    private SingleJointedArmSim m_armSim = new SingleJointedArmSim(
        m_armGearbox,
        Constants.kArmReduction,
        SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
        Constants.kArmLength,
        Constants.kMinAngleRads,
        Constants.kMaxAngleRads,
        true,
        VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );

    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    public PivotWristIOSim() {
        m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    }

    @Override
    public void setPosition(double setpoint) {
        state = PivotWristIO.State.PID;

        this.setpoint = setpoint;
    }

    @Override
    public void manualControl(double newSpeed) {
        state = PivotWristIO.State.MANUAL;

        speed = newSpeed;
    }

    @Override
    public void updateIO() {
        if (state == PivotWristIO.State.MANUAL) {
            m_motor.setVoltage(speed * RobotController.getBatteryVoltage());
        } else {
            double pidOutput = m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(setpoint));
            m_motor.setVoltage(pidOutput);
        }
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_armSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setDistance(m_armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    }

    @Override
    public double getWristAngle() {
        return Units.radiansToDegrees(m_armSim.getAngleRads());
    }

    public void stop() {
        m_motor.set(0.0);
    }

    @Override
    public void displayShuffleboardIO() {
    }

    @Override
    public void tuningInitIO() {
    }

    @Override
    public void tuningPeriodicIO() {
    }

    @Override
    public PivotWristIO.State getState() {
        return state;
    }
}
