package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Delay;
import frc.robot.commands.claw.ClawCloseCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;


public class ChuckCubeKickCommand extends SequentialCommandGroup {
    // TODO Do this
    public ChuckCubeKickCommand(Elevator elevator, PivotArm pivotArm, Claw claw) {
        double beautifulTimeConstant= 0.2;

        // Use addRequirements() here to declare subsystem dependencies.
        addCommands(
            new HoldCommand(elevator, pivotArm),
            new ParallelCommandGroup(
                new PivotArmPIDCommand(pivotArm, 95),
                new Delay(0.5).andThen(new ElevatorPIDCommand(elevator, -ELEVATOR_SETPOINT_EXTEND)),
                new Delay(0.5 + beautifulTimeConstant).andThen(new ClawOpenCommand(claw, 0.15))
            )
        );

        addRequirements(elevator, pivotArm, claw);
    }
}
