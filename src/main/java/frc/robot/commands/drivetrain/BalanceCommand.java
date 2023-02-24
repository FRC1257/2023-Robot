package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Gyro;

public class BalanceCommand extends CommandBase {

  private Drivetrain drivetrain;

  private double error;
  private double currentAngle;
  private double drivePower;
  private Gyro gyro = Gyro.getInstance();

  private int levelCounter; 

  public BalanceCommand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.currentAngle = gyro.getRollAngle();

    if (Math.abs(error) < Autonomous.BALANCE_THRESHOLD_DEGREES) {
      levelCounter ++;
    } else {
      levelCounter = 0;
    }

    error = Autonomous.BALANCE_SETPOINT_ANGLE - currentAngle;
    drivePower = -Math.min(Autonomous.BALANCE_KP * error, 1);
    // Limit the max power
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }
    drivetrain.velocityDrive(drivePower, 0);

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.velocityDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return levelCounter > Autonomous.BALANCE_STEPS_THRESHOLD;
  }
}