package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawCubeStateCommand;
import frc.robot.commands.claw.ClawEjectCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.PivotWrist;

import static frc.robot.Constants.ElevatorConstants;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_UP;


public class ScoreCommand extends SequentialCommandGroup {
    // TODO Do this
    public ScoreCommand(Elevator elevator, PivotArm pivotArm, PivotWrist pivotWrist, Claw claw) {
        // Use addRequirements() here to declare subsystem dependencies.
        addCommands(
            new ElevatorPIDCommand(elevator, ElevatorConstants.ELEVATOR_SETPOINT_EXTEND),
            new PivotArmPIDCommand(pivotArm, PIVOT_ARM_SETPOINT_UP),
            new ClawCubeStateCommand(claw),
            new ClawEjectCommand(claw)
        );
    }
}
