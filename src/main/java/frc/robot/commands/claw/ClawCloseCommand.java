package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import static frc.robot.Constants.Claw.*;

public class ClawCloseCommand extends CommandBase {
    private Claw claw;
    double start, time;

    public ClawCloseCommand(Claw claw, double time) {
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
        claw.manualControl(CLAW_CLOSED_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        claw.manualControl(0);
    }

    @Override
    public boolean isFinished() {
        if (RobotBase.isSimulation())
            return true;
        return Timer.getFPGATimestamp() - start >= time;
    }
}