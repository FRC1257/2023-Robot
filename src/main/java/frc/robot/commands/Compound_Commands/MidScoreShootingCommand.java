package frc.robot.commands.Compound_Commands;

import frc.robot.commands.Intake.IntakeEjectingCommand;
import frc.robot.commands.Intake.IntakeShootingCommand;
import frc.robot.commands.IntakeArm.IntakeArmPIDCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_SETPOINT_MID;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class MidScoreShootingCommand extends SequentialCommandGroup {
    
    // TODO: Add new command sequence based off of Cube Shooting
    public MidScoreShootingCommand(IntakeArm intakeArm, Intake intake) {
        addCommands(
            new IntakeArmPIDCommand(intakeArm, PIVOT_ARM_SETPOINT_MID),
            new IntakeShootingCommand(intake)
        );
    }
}