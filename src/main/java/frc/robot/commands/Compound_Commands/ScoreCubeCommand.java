package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Delay;
import frc.robot.commands.claw.ClawCloseCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;


public class ScoreCubeCommand extends SequentialCommandGroup {
    // TODO Do this
    public ScoreCubeCommand(Elevator elevator, PivotArm pivotArm, Claw claw) {
        // Use addRequirements() here to declare subsystem dependencies.
        addCommands(
            new ParallelDeadlineGroup(
                new MidCubeSetpointCommand(elevator, pivotArm),
                new ClawCloseCommand(claw, 5)
            ),
            new Delay(0.2),
            new ClawOpenCommand(claw, 0.5)
        );
    }
}
