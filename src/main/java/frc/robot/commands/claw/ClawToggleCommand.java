package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMotionState;
import frc.robot.subsystems.Claw.ClawState;

import static frc.robot.Constants.Claw.*;

public class ClawToggleCommand extends CommandBase {
    private Claw claw;
    private ClawState clawState;

    public ClawToggleCommand(Claw claw) {
        this.claw = claw;
        clawState = claw.getState();
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        switch(clawState) {
            case OPEN:
                claw.close();
                break;
            case CLOSED:
                claw.open();
                break;
        }
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