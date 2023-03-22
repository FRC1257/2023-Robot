package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;

public class ResetPIDCommand extends CommandBase {
    private PivotArm pivotArm;
    private Elevator elevator;

    public ResetPIDCommand(Elevator elevator, PivotArm pivotArm) {
        this.pivotArm = pivotArm;
        this.elevator = elevator;

        addRequirements(elevator, pivotArm);
    }

    @Override
    public void initialize() {
        pivotArm.endPID();
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
