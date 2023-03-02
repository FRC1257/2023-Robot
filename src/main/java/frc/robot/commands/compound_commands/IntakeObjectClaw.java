package frc.robot.commands.compound_commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawIntakeCommand;
import frc.robot.commands.elevator.ElevatorExtendCommand;
import frc.robot.commands.intake.IntakeIntakingCommand;
import frc.robot.commands.intakearm.IntakeArmPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakeObjectClaw extends SequentialCommandGroup{
    public IntakeObjectClaw(PivotArm pivotarm, PivotWrist pivotwrist, Intake intake, IntakeArm intakeArm, Elevator elevator, Claw claw) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ParallelCommandGroup( // run intake entire time
                new ElevatorExtendCommand(elevator),
                new SequentialCommandGroup(
                    new PivotArmPIDCommand(pivotarm, Constants.PivotArm.PIVOT_ARM_SETPOINT_INTAKE),
                    new PivotWristPIDCommand(pivotwrist, Constants.PivotWrist.WRIST_SETPOINT_INTAKE),
                    new ClawIntakeCommand(claw)
                )
                
                )
               
            );
    }

 
}
