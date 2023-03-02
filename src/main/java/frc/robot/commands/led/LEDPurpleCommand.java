package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class LEDPurpleCommand extends CommandBase {

    LED led;

    public LEDPurpleCommand(LED led) {
        this.led = led;
        addRequirements(led);
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        led.purple();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
