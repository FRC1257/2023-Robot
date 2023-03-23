package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_MID;
import static frc.robot.Constants.PivotWrist.WRIST_SETPOINT_MID;

import frc.robot.subsystems.Intake;
import frc.robot.commands.Intake.IntakeEjectingCommand;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Claw;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SETPOINT_EXTEND;


public class MidScoreCommand extends SequentialCommandGroup {
    
    // TODO: Add new command sequence based off of Cube Shooting
    public MidScoreCommand(Elevator elevator, PivotArm pivotarm, Claw claw, Intake intake) {
        addCommands(
            new ElevatorPIDCommand(elevator, ELEVATOR_SETPOINT_EXTEND), 
            new PivotArmPIDCommand(pivotarm, PIVOT_ARM_SETPOINT_MID),
            new IntakeEjectingCommand(intake)
        );
    }
}