package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Delay;
import frc.robot.commands.claw.ClawCloseCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;


public class ChuckCubeCommand extends SequentialCommandGroup {
    // TODO Do this
    public ChuckCubeCommand(Elevator elevator, PivotArm pivotArm, Claw claw) {
        double beautifulTimeConstant= 1;

        // Use addRequirements() here to declare subsystem dependencies.
        addCommands(
            new HoldCommand(elevator, pivotArm),
            new ParallelDeadlineGroup(
                new Delay(beautifulTimeConstant).andThen(new ClawOpenCommand(claw, 0.15)), 
                new MidCubeSetpointCommand(elevator, pivotArm)
            )
            
        );

        addRequirements(elevator, pivotArm, claw);
    }
}
