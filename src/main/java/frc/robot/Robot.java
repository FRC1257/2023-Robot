/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SnailController;
import frc.robot.util.TunableNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This sample program shows how to read basic bus measurements from the
 * SparkMax
 */
public class Robot extends TimedRobot {
    /**
     * Parameters for the SparkMax are defined below. Be sure to change the deviceID
     * and motor type to match your setup.
     */
    private static final int deviceID = 1;
    private static TunableNumber device = new TunableNumber("Device ID", deviceID);
    private static final MotorType motorType = MotorType.kBrushless;
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    /**
     * A single joystick will be used to control motor outputs when the robot is
     * enabled.
     */
    private SnailController m_controller;

    @Override
    public void robotInit() {
        m_motor = new CANSparkMax(deviceID, motorType);
        m_encoder = m_motor.getEncoder();

        m_controller = new SnailController(0);
    }

    @Override
    public void robotPeriodic() {
        if (device.checkUpdate()) {
            DriverStation.reportError("Changing Motor to " + (int) device.get(), false);
            m_motor = new CANSparkMax((int) device.get(), motorType);
            m_encoder = m_motor.getEncoder();
        }

        /**
         * There are several useful bus measurements you can get from the SparkMax.
         * This includes bus voltage (V), output current (A), Applied Output
         * (duty cycle), and motor temperature (C)
         */
        double busVoltage = m_motor.getBusVoltage();
        double current = m_motor.getOutputCurrent();
        double appliedOut = m_motor.getAppliedOutput();
        double temperature = m_motor.getMotorTemperature();
        double position = m_encoder.getPosition();

        // Open SmartDashboard when your program is running to see the values
        SmartDashboard.putNumber("A Motor Bus Voltage", busVoltage);
        SmartDashboard.putNumber("A Motor Current", current);
        SmartDashboard.putNumber("A Motor Applied Output", appliedOut);
        SmartDashboard.putNumber("A Motor Motor Temperature", temperature);
        SmartDashboard.putNumber("A Motor Position", position);
    }

    @Override
    public void teleopPeriodic() {
        m_motor.set(m_controller.getLeftY());
    }
}