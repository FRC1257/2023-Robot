package frc.robot.commands.Compound_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeShootingCommand;
import frc.robot.commands.IntakeArm.IntakeArmPIDCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import static frc.robot.Constants.IntakeArmConstants.INTAKE_SETPOINT_BOT;

public class LowScoreShootingCommand extends SequentialCommandGroup {

    public LowScoreShootingCommand(Intake intake, IntakeArm intakeArm) {
        addCommands(
            new ParallelCommandGroup(
                new IntakeShootingCommand(intake),
                new IntakeArmPIDCommand(intakeArm, INTAKE_SETPOINT_BOT)
            ),
        new IntakeShootingCommand(intake)
        );
    }
    
}
