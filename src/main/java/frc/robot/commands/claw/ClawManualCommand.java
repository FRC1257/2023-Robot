package frc.robot.commands.claw;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawManualCommand extends CommandBase {
    private Claw claw;
    private DoubleSupplier speedSupplier;

    public ClawManualCommand(Claw claw, DoubleSupplier speedSupplier) {
        this.claw = claw;
        this.speedSupplier = speedSupplier;
        addRequirements(claw);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        if (speed < 0) {
            // closing
            claw.manualControl(speed/6);
        } else {
            // opening
            claw.manualControl(speed/7);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
