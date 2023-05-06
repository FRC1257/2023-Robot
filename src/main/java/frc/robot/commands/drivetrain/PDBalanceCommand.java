package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.Autonomous;
import frc.robot.util.Gyro;
import frc.robot.util.TunableNumber;
import edu.wpi.first.math.MathUtil;

public class PDBalanceCommand extends CommandBase {
	private final Drivetrain drivetrain;
	private PIDController controller;
	private final Gyro gyro;
	private double error;
	private boolean stop;
	private int levelCounter = 0;

	private TunableNumber kP = new TunableNumber("Balance kP", Autonomous.BALANCE_KP);
	private TunableNumber maxOutput = new TunableNumber("Balance kD", 0.75);

	public PDBalanceCommand(Drivetrain drivetrain, boolean stop) {
		this.stop = stop;

		this.drivetrain = drivetrain;
		this.controller = new PIDController(kP.get(), 0, Autonomous.BALANCE_KD);
		this.controller.setTolerance(Autonomous.BALANCE_THRESHOLD_DEGREES);
		gyro = Gyro.getInstance();
		error = gyro.getRollAngle();
		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		this.controller = new PIDController(kP.get(), 0, Autonomous.BALANCE_KD);
		this.controller.setTolerance(Autonomous.BALANCE_THRESHOLD_DEGREES);
		drivetrain.setSlowMode(true);
	}

	@Override
	public void execute() {
		error = gyro.getRollAngle();

		double velocity = controller.calculate(error, Autonomous.BALANCE_SETPOINT_ANGLE);
		SmartDashboard.putBoolean("Balance Running", true);
		error = gyro.getRollAngle();
		SmartDashboard.putNumber("Balance Error", error);
        
		SmartDashboard.putNumber("Balance Unclamped velocity", velocity);
		if (controller.atSetpoint()) {
			drivetrain.velocityDrive(-MathUtil.clamp(velocity, -0.1, 0.1), 0);
			levelCounter ++;
		} else {
			levelCounter = 0;
			drivetrain.velocityDrive(-MathUtil.clamp(velocity, -maxOutput.get(), maxOutput.get()), 0);
		}
		
	}

	@Override
	public void end(boolean interrupted) {
		SmartDashboard.putBoolean("Balance Running", false);
		drivetrain.velocityDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		if (!stop) 
			return false;
		if (controller.atSetpoint()) {
			return levelCounter > 100;
		}

		return false;
	}
}