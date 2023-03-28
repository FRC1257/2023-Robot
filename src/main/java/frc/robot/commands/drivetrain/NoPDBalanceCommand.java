package frc.robot.commands.drivetrain;

// thank you 6002
// our own balance command wasn't working
// https://github.com/team6002/FRC2023ChargedUp/blob/main/src/main/java/frc/robot/commands/CMD_AdjustBalance.java

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Autonomous;
import frc.robot.util.Gyro;
import frc.robot.util.TunableNumber;

public class NoPDBalanceCommand extends CommandBase {
	private final Drivetrain drivetrain;
	private final Gyro gyro;
    private Timer m_timer = new Timer();
	private double error;
	private double m_timeLimit = 5;

    private TunableNumber speed = new TunableNumber("Balance Speed Tune", .17);

	public NoPDBalanceCommand(Drivetrain drivetrain) {

		this.drivetrain = drivetrain;

		gyro = Gyro.getInstance();
		error = gyro.getRollAngle();
		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
        m_timer.start();
        m_timer.reset();
        if (Math.abs(error) < 5){
            m_timeLimit = Math.abs(error * 0.04);  
        } else{ 
            m_timeLimit = Math.abs(error * 0.07);  
        }

		drivetrain.setSlowMode(true);
	}

	@Override
	public void execute() {
		error = gyro.getRollAngle();
        
        drivetrain.velocityDrive(Math.copySign(speed.get(), error), 0);
		
		SmartDashboard.putBoolean("Align error", m_timer.get() > m_timeLimit);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.velocityDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		return (m_timer.get() > m_timeLimit);
	}
}