package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private String key;
    private double defaultValue;
    private double value;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;

        SmartDashboard.putNumber(key, defaultValue);
    }

    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    public void set(double value) {
        this.value = value;
        SmartDashboard.putNumber(key, value);
    }

    public void reset() {
        set(defaultValue);
    }

    public boolean checkUpdate() {
        double newValue = get();
        if (newValue != value) {
            value = newValue;
            return true;
        }
        return false;
    }

    public void updateFunction(Runnable function) {
        if (checkUpdate()) {
            function.run();
        }
    }
}