package frc.robot.commands.compound_commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorExtendCommand;
import frc.robot.commands.intake.IntakeIntakingCommand;
import frc.robot.commands.intakearm.IntakeArmPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakeObjectClaw extends SequentialCommandGroup{
    public IntakeObjectClaw(PivotArm pivotarm, PivotWrist pivtowrist, Intake intake, IntakeArm intakeArm, Elevator elevator) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ParallelCommandGroup(
                new IntakeIntakingCommand(intake), // run intake entire time
                new ElevatorExtendCommand(elevator),
                new SequentialCommandGroup(
                    new IntakeArmPIDCommand(intakeArm, Constants.IntakeArm.INTAKE_SETPOINT_BOT)
                )
            )
        );
    }

 
}
