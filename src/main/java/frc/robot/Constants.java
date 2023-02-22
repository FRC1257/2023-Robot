package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
    };


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
        public static double DRIVE_TRAJ_MAX_ACC = 0.950; //.75;  // m/s^2
        public static double DRIVE_TRAJ_RAMSETE_B = 2.0;
        public static double DRIVE_TRAJ_RAMSETE_ZETA = 0.7;

        // aligning
        public static double DRIVE_ALIGN_MAX_VEL = 0.75; // m/s
        public static double DRIVE_ALIGN_MAX_ACC = 0.350; //.75;  // m/s^2

        // linear position PID
        public static double[] DRIVE_DIST_PID = { 3.50, 0.0, 0.0 };
        public static double DRIVE_DIST_ANGLE_P = 0.1;
        public static double DRIVE_DIST_TOLERANCE = 0.01;
        public static double DRIVE_DIST_MAX_OUTPUT = 0.6;

        // angular position PID
        public static double[] DRIVE_ANGLE_PID = { 0.1, 0.0, 0.01 };
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
    };


    public static class VisionConstants {
        public static double VISION_KP = 0.02;
        public static double VISION_FEEDFORWARD = 0.01;
        public static double TRACKED_TAG_ROTATION_KP = 0.0175;

        public static Transform3d CAMERA_TO_ROBOT_FRONT = new Transform3d(new Translation3d(0.35, 0.15, 0.1), new Rotation3d(0, 0, 0));
        public static Transform3d CAMERA_TO_ROBOT_BACK = new Transform3d(new Translation3d(0.35, -0.15 , 0.1), new Rotation3d(0.0, 180.0, 0.0));
        
        // public static AprilTagFieldLayout aprilTagFieldLayout = new
        // AprilTagFieldLayout(AprilTagFields.kDefaultField.m_resourceFile);
        public static String USB_CAMERA_NAME_FRONT = "Front Camera";
        public static String USB_CAMERA_NAME_BACK = "Back Camera";
    };


    public static class Autonomous {
        // all of these positions have been estimated using PathWeaver
        // TODO calculate true positions

        public static Pose2d[] BLUE_SCORE_POSE = new Pose2d[] {
                new Pose2d(1.425, 0.453, Rotation2d.fromDegrees(0)), // Score location 1 on blue side
                new Pose2d(1.425, 1.044, Rotation2d.fromDegrees(0)), // score 2
                new Pose2d(1.425, 1.579, Rotation2d.fromDegrees(0)), // 3
                new Pose2d(1.425, 2.204, Rotation2d.fromDegrees(0)), // ...
                new Pose2d(1.425, 2.773, Rotation2d.fromDegrees(0)),
                new Pose2d(1.425, 3.273, Rotation2d.fromDegrees(0)),
                new Pose2d(1.425, 3.831, Rotation2d.fromDegrees(0)),
                new Pose2d(1.425, 4.433, Rotation2d.fromDegrees(0)),
                new Pose2d(1.425, 5.082, Rotation2d.fromDegrees(0))
        };

        public static Pose2d[] RED_SCORE_POSE = new Pose2d[] {
                new Pose2d(15.15, 0.453, Rotation2d.fromDegrees(180)), // Score location 1 on blue side
                new Pose2d(15.15, 1.044, Rotation2d.fromDegrees(180)), // score 2
                new Pose2d(15.15, 1.579, Rotation2d.fromDegrees(180)), // 3
                new Pose2d(15.15, 2.204, Rotation2d.fromDegrees(180)), // ...
                new Pose2d(15.15, 2.773, Rotation2d.fromDegrees(180)),
                new Pose2d(15.15, 3.273, Rotation2d.fromDegrees(180)),
                new Pose2d(15.15, 3.831, Rotation2d.fromDegrees(180)),
                new Pose2d(15.15, 4.433, Rotation2d.fromDegrees(180)),
                new Pose2d(15.15, 5.082, Rotation2d.fromDegrees(180))
        };

        public static Pose2d[] BLUE_CARGO_POSE = new Pose2d[] {
                new Pose2d(7.066, 0.896, Rotation2d.fromDegrees(0)),
                new Pose2d(7.066, 2.125, Rotation2d.fromDegrees(0)),
                new Pose2d(7.066, 3.353, Rotation2d.fromDegrees(0)),
                new Pose2d(7.066, 4.602, Rotation2d.fromDegrees(0)),
        };

        public static Pose2d[] RED_CARGO_POSE = new Pose2d[] {
                new Pose2d(9.5, 0.896, Rotation2d.fromDegrees(180)),
                new Pose2d(9.5, 2.125, Rotation2d.fromDegrees(180)),
                new Pose2d(9.5, 3.353, Rotation2d.fromDegrees(180)),
                new Pose2d(9.5, 4.602, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d[] BLUE_WAYPOINT_POSE = new Pose2d[] {
                new Pose2d(2.87, 4.73, Rotation2d.fromDegrees(0)),
                new Pose2d(4.8, 4.73, Rotation2d.fromDegrees(0)),
                new Pose2d(2.87, 0.754, Rotation2d.fromDegrees(0)),
                new Pose2d(4.8, 0.754, Rotation2d.fromDegrees(0)),
        };
        public static Pose2d[] RED_WAYPOINT_POSE = new Pose2d[] {
                new Pose2d(11.6, 4.73, Rotation2d.fromDegrees(0)),
                new Pose2d(13.5, 4.73, Rotation2d.fromDegrees(0)),
                new Pose2d(11.6, 0.754, Rotation2d.fromDegrees(0)),
                new Pose2d(13.5, 0.754, Rotation2d.fromDegrees(0)),
        };

        public static Pose2d BLUE_CHARGE_POSE = new Pose2d(3.89, 2.75, Rotation2d.fromDegrees(0));
        public static Pose2d RED_CHARGE_POSE = new Pose2d(12.58, 2.75, Rotation2d.fromDegrees(0));

        // bottom to top (farthest from community to closest)
        public static Pose2d[] BLUE_START_POSE = new Pose2d[] {
                new Pose2d(2.285, 0.736, Rotation2d.fromDegrees(0)),
                new Pose2d(2.285, 2.638, Rotation2d.fromDegrees(0)),
                new Pose2d(2.285, 4.357, Rotation2d.fromDegrees(0)),
        };

        public static Pose2d[] RED_START_POSE = new Pose2d[] {
                new Pose2d(14.384, 0.736, Rotation2d.fromDegrees(180)),
                new Pose2d(14.384, 2.638, Rotation2d.fromDegrees(180)),
                new Pose2d(14.384, 4.357, Rotation2d.fromDegrees(180)),
        };
        public static double CHARGE_STATION_LOWER_Y = 1.508506;
        public static double CHARGE_STATION_UPPER_Y = 3.978656;

        public static double CHARGE_CENTER_Y = 2.75;

    };


    public static double PI = 3.14159265;
    public static double UPDATE_PERIOD = 0.010; // seconds
    public final static int NEO_550_CURRENT_LIMIT = 25; // amps
    public final static int NEO_CURRENT_LIMIT = 80; // amps
    
}