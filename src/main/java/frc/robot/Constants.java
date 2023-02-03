package frc.robot;

import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * Each subsystem should have its own static inner class to hold its constants
 */
public final class Constants {

    public static class ElectricalLayout {
        public final static int CONTROLLER_DRIVER_ID = 0;
        public final static int CONTROLLER_OPERATOR_ID = 1;

        // Drivetrain
        public final static int DRIVE_FRONT_LEFT = 17;
        public final static int DRIVE_FRONT_RIGHT = 13;
        public final static int DRIVE_BACK_LEFT = 5;
        public final static int DRIVE_BACK_RIGHT = 2;

        public final static int DOUBLE_SOLENOID_FORWARD = 0;
        public final static int DOUBLE_SOLENOID_REVERSE = 1;
    }

    public static class Drivetrain {
        // drivetrain constants
        public static double DRIVE_TRACK_WIDTH_M = 0.66; // m
        public static double DRIVE_WHEEL_DIAM_M = 0.1524; // m
        public static double DRIVE_GEARBOX_REDUCTION = 10.71;

        // driving modifiers
        public static double DRIVE_SLOW_TURN_MULT = 0.25;
        public static double DRIVE_SLOW_FORWARD_MULT = 0.25;

        // closed loop driving
        public static double DRIVE_CLOSED_MAX_VEL = 4.0; // m/s
        public static double DRIVE_CLOSED_MAX_ROT_TELEOP = 360.00; //
        public static double DRIVE_CLOSED_MAX_ROT_AUTO = 100.0; // deg/s
        public static double DRIVE_CLOSED_MAX_ACC = 1.5; // m/s^2

        // trajectory following
        public static double DRIVE_TRAJ_MAX_VEL = 1.0; // m/s
        public static double DRIVE_TRAJ_MAX_ACC = 1.25; // m/s^2
        public static double DRIVE_TRAJ_RAMSETE_B = 2.0;
        public static double DRIVE_TRAJ_RAMSETE_ZETA = 0.7;

        // linear position PID
        public static double[] DRIVE_DIST_PID = {3.50, 0.0, 0.0};
        public static double DRIVE_DIST_ANGLE_P = 0.1;
        public static double DRIVE_DIST_TOLERANCE = 0.01;
        public static double DRIVE_DIST_MAX_OUTPUT = 0.6;

        // angular position PID
        public static double[] DRIVE_ANGLE_PID = {0.1, 0.0, 0.01};
        public static double DRIVE_ANGLE_TOLERANCE = 0.075;
        public static double DRIVE_ANGLE_MAX_OUTPUT = 0.5;

        // velocity PID (for closed loop, profiling, and trajectory)
        public static int DRIVE_VEL_SLOT = 0;
        public static double DRIVE_VEL_LEFT_P = 0.25;
        public static double DRIVE_VEL_LEFT_F = 0.25;
        public static double DRIVE_VEL_RIGHT_P = 0.25;
        public static double DRIVE_VEL_RIGHT_F = 0.25;

        // profiling position PID (for further refinement of tracking)
        public static double DRIVE_PROFILE_LEFT_P = 0.1;
        public static double DRIVE_PROFILE_RIGHT_P = 0.1;

        // vision PID
        public static final double TRACKED_TAG_ROTATION_KP = 0.375;
        public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3; // P (Proportional) constant of a PID loop
        public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2; // P (Proportional) constant of a PID loop
        public static final double APRILTAG_POWER_CAP = 0.75;
    }

    public static class Vision {
        public static double VISION_KP = 0.02;
        public static double VISION_FEEDFORWARD = 0.01;
        public static double TRACKED_TAG_ROTATION_KP = 0.0175;
    }

    public static class Autonomous {
        
    }

    public static double PI = 3.14159265;
    public static double UPDATE_PERIOD = 0.010; // seconds
    public final static int NEO_550_CURRENT_LIMIT = 25; // amps
    public final static int NEO_CURRENT_LIMIT = 80; // amps

    public static String USB_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
}
