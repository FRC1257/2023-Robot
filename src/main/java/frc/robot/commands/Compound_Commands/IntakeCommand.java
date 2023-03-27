package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorPIDCommand;
import frc.robot.commands.pivotArm.PivotArmPIDCommand;
import frc.robot.commands.Intake.IntakeIntakingCommand;
import frc.robot.commands.IntakeArm.IntakeArmPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import static frc.robot.Constants.IntakeArmConstants.INTAKE_SETPOINT_BOT;

public class IntakeCommand extends ParallelCommandGroup{
    public IntakeCommand(Intake intake, IntakeArm intakeArm) {
        addCommands(
            new IntakeArmPIDCommand(intakeArm, INTAKE_SETPOINT_BOT),
            new IntakeIntakingCommand(intake)       
        );
    }
}