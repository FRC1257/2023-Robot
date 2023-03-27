package frc.robot.commands.IntakeArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmManualCommand extends CommandBase{
	private IntakeArm intakearm;
    private DoubleSupplier speedSupplier;
	public IntakeArmManualCommand(IntakeArm intakearm, DoubleSupplier speedSupplier) {
        this.intakearm = intakearm;
        this.speedSupplier = speedSupplier;
        addRequirements(intakearm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intakearm.manual(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}