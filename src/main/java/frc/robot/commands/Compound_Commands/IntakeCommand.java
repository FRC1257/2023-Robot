package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawIntakeCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.commands.pivotWrist.PivotWristPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;

public class IntakeCommand extends SequentialCommandGroup{
    public IntakeCommand(PivotArm pivotarm, PivotWrist pivotwrist, Elevator elevator, Claw claw) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ParallelCommandGroup( // run intake entire time
                new ElevatorPIDCommand(elevator, ELEVATOR_SETPOINT_EXTEND),
                new SequentialCommandGroup(
                    new PivotArmPIDCommand(pivotarm, Constants.PivotArm.PIVOT_ARM_SETPOINT_INTAKE),
                    new PivotWristPIDCommand(pivotwrist, Constants.PivotWrist.WRIST_SETPOINT_INTAKE),
                    new ClawIntakeCommand(claw)
                )   
            )
        );
    }

 
}