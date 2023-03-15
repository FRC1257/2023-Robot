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

public class HoldCommand extends SequentialCommandGroup{
    public HoldCommand(PivotArm pivotarm, PivotWrist pivotwrist, Elevator elevator, Claw claw) {
        addCommands( //WITH INTAKE
            //new ElevatorExtendCommand(elevator),
            new ParallelCommandGroup( // run intake entire time
                new ElevatorPIDCommand(elevator, 0),
                new SequentialCommandGroup(
                    new PivotArmPIDCommand(pivotarm, 0),
                    new PivotWristPIDCommand(pivotwrist, 0),
                    new ClawIntakeCommand(claw)
                )   
            )
        );
    }

 
}