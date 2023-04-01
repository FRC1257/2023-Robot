package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BrakeCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private Timer m_timer = new Timer();

	public BrakeCommand(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		drivetrain.setSlowMode(true);
        m_timer.start();
        m_timer.reset();
        drivetrain.setStopBrake();
	}

	@Override
	public void execute() {
        drivetrain.manualDrive(0, 0);
    }

	@Override
	public void end(boolean interrupted) {
		drivetrain.setNormalBrake();
	}

	@Override
	public boolean isFinished() {
		return (m_timer.get() > 2);
	}
}
