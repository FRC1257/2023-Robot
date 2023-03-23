package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class LEDYellowCommand extends CommandBase {
    private LED led;

    public LEDYellowCommand(LED led) {
        this.led = led;
        addRequirements(led);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        led.yellow();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
