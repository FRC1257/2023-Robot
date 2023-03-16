package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.PivotWrist;

public class ResetPIDCommand extends CommandBase {
    private PivotArm pivotArm;
    private PivotWrist pivotWrist;
    private Elevator elevator;

    public ResetPIDCommand(Elevator elevator, PivotArm pivotArm, PivotWrist pivotWrist) {
        this.pivotArm = pivotArm;
        this.pivotWrist = pivotWrist;
        this.elevator = elevator;

        addRequirements(elevator, pivotArm, pivotWrist);
    }

    @Override
    public void initialize() {
        pivotArm.endPID();
        pivotWrist.endPID();
        elevator.endPID();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
