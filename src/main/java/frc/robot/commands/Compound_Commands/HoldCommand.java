package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeNeutralCommand;
import frc.robot.commands.IntakeArm.IntakeArmPIDCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;

public class HoldCommand extends ParallelCommandGroup{
    public HoldCommand(Elevator elevator, PivotArm pivotarm, IntakeArm intakearm, Intake intake) {
        addCommands(
            new PivotArmPIDCommand(pivotarm, 0),
            new IntakeArmPIDCommand(intakearm, 0),
            new IntakeNeutralCommand(intake),
            new ElevatorPIDCommand(elevator, 0)
        );
    }

 
}