package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Autonomous;
import frc.robot.util.Gyro;
public class PDBalanceCommand extends CommandBase {
	private final Drivetrain drivetrain;
	private final PIDController controller;
	private final Gyro gyro;
	private double error = 1.0;
	public PDBalanceCommand(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		this.controller = new PIDController(Autonomous.BALANCE_KP, 0, Autonomous.BALANCE_KD);
		this.controller.setTolerance(Autonomous.BALANCE_THRESHOLD_DEGREES);

		gyro = Gyro.getInstance();
		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double velocity = controller.calculate(gyro.getPitchAngle(), Autonomous.BALANCE_SETPOINT_ANGLE);
		drivetrain.velocityDrive(-Math.min(velocity, 1), 0);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.velocityDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(error) < Autonomous.BALANCE_THRESHOLD_DEGREES;
	}
}
