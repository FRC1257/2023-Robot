package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.Claw.*;

public class ClawOpenCommand extends CommandBase {
    private Claw claw;

    public ClawOpenCommand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.open();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        claw.manualControl(0);
    }

    @Override
    public boolean isFinished() {
        return claw.atSetpoint();
    }
}