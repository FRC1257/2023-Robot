package frc.robot.commands.pivotarm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotArm;

public class PivotArmManualCommand extends CommandBase {
    private PivotArm pivotArm;
    private DoubleSupplier speedSupplier;

    public PivotArmManualCommand(PivotArm pivotArm, DoubleSupplier speedSupplier) {
        this.pivotArm = pivotArm;
        this.speedSupplier = speedSupplier;
        addRequirements(pivotArm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivotArm.manualControl(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}