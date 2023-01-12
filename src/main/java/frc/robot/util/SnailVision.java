package frc.robot.util;
// TODO add photonvision
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SnailSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.Vision.*;

public class SnailVision {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision");
    private static int pipeline = 0;
    

    public static void init() {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public static void printData() {
        // System.out.println("I'm some data " + table.getEntry("tx").getDouble(0));
    }

    public static double getTargetX() {
        return 3;
    }

    public static double getTargetY() {
        return 3;
    }

    public static double getTargetArea() {
        return 30;
    }

    public static boolean isTargetValid () {
        return true;
    }

    public static double getVisionAdd() {
        double visionAdd = 0.0;

        if (isTargetValid()) { 
            visionAdd = Math.copySign(VISION_FEEDFORWARD, getTargetX());
            visionAdd += VISION_KP * getTargetX();
        }

        return visionAdd;
    }

    public static void setConstantTuning() {
        SmartDashboard.putNumber("Vision Feedforward", VISION_FEEDFORWARD);
        SmartDashboard.putNumber("Vision kP", VISION_KP);
    }

    public static void getConstantTuning() {
        VISION_FEEDFORWARD = SmartDashboard.getNumber("Vision Feedforward", VISION_FEEDFORWARD);
        VISION_KP = SmartDashboard.getNumber("Vision kP", VISION_KP);
    }
    
}
