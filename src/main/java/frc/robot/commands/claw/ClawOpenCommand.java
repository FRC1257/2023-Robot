package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.Claw.*;

public class ClawOpenCommand extends CommandBase {
    private Claw claw;
    double start, time;

    public ClawOpenCommand(Claw claw, double time) {
        this.claw = claw;
        this.time = time;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        claw.manualControl(CLAW_OPEN_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        claw.manualControl(0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - start >= time;
    }
}