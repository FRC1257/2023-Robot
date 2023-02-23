// need to bind command
package frc.robot.commands.pivotWrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotWrist;

public class PivotWristManualCommand extends CommandBase {

    private PivotWrist pivotWrist;
    private DoubleSupplier speedSupplier;

    public PivotWristManualCommand(PivotWrist pivotWrist, DoubleSupplier speedSupplier) {
        this.pivotWrist = pivotWrist;
        this.speedSupplier = speedSupplier;

        addRequirements(pivotWrist);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivotWrist.manual(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
