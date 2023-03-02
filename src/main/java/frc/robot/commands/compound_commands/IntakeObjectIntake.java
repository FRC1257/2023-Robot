package frc.robot.commands.compound_commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorRetractCommand;
import frc.robot.commands.intake.IntakeIntakingCommand;
import frc.robot.commands.intakearm.IntakeArmPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakeObjectIntake extends SequentialCommandGroup{

    public IntakeObjectIntake(PivotArm pivotarm, PivotWrist pivotwrist, Intake intake, IntakeArm intakeArm, Elevator elevator) {
        addCommands( //WITH INTAKE
            new ElevatorRetractCommand(elevator),
            new ParallelCommandGroup(
                new IntakeIntakingCommand(intake), // run intake entire time
                new SequentialCommandGroup(
                    new IntakeArmPIDCommand(intakeArm, Constants.IntakeArm.INTAKE_SETPOINT_BOT),
                    new PivotArmPIDCommand(pivotarm, Constants.PivotArm.PIVOT_ARM_SETPOINT_INTAKE),
                    new PivotWristPIDCommand(pivotwrist, Constants.PivotWrist.WRIST_SETPOINT_INTAKE)
                )
            )
        );
    }
}
