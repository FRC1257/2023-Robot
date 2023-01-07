package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxTrigger extends Trigger {
    
    private final XboxController controller;
    private boolean leftHand;

    public XboxTrigger(XboxController controller, boolean leftHand) {
        this.controller = controller;
        this.leftHand = leftHand;
    }

    @Override
    public boolean get() {
        if (leftHand) return controller.getLeftTriggerAxis() > 0.5; 
        else return controller.getRightTriggerAxis() > 0.5;
    }
}
