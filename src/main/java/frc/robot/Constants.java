package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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

        public static final class DriveConstants {
                public static final int kLeftMotor1Port = 0;
                public static final int kLeftMotor2Port = 1;
                public static final int kRightMotor1Port = 2;
                public static final int kRightMotor2Port = 3;
                
                public static final int[] kLeftEncoderPorts = new int[] {0, 1};
                public static final int[] kRightEncoderPorts = new int[] {2, 3};
                public static final boolean kLeftEncoderReversed = false;
                public static final boolean kRightEncoderReversed = true;
                
                public static final double kTrackwidthMeters = 0.69;
                public static final DifferentialDriveKinematics kDriveKinematics =
                        new DifferentialDriveKinematics(kTrackwidthMeters);
                
                public static final int kEncoderCPR = 1024;
                public static final double kWheelDiameterMeters = 0.15;
                public static final double kEncoderDistancePerPulse =
                        // Assumes the encoders are directly mounted on the wheel shafts
                        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
                
                public static final boolean kGyroReversed = true;
                
                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or theoretically
                // for *your* robot's drive.
                // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
                // values for your robot.
                public static final double ksVolts = 0.22;
                public static final double kvVoltSecondsPerMeter = 1.98;
                public static final double kaVoltSecondsSquaredPerMeter = 0.2;
                
                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or theoretically
                // for *your* robot's drive.
                // These two values are "angular" kV and kA
                public static final double kvVoltSecondsPerRadian = 1.5;
                public static final double kaVoltSecondsSquaredPerRadian = 0.3;
                
                public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                        LinearSystemId.identifyDrivetrainSystem(
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian,
                        kaVoltSecondsSquaredPerRadian);
                
                // Example values only -- use what's on your physical robot!
                public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
                public static final double kDriveGearing = 8;
                
                // Example value only - as above, this must be tuned for your drive!
                public static final double kPDriveVel = 8.5;
              }
            
        public static final class OIConstants {
                public static final int kDriverControllerPort = 0;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                
                // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
                public static final double kRamseteB = 2;
                public static final double kRamseteZeta = 0.7;
        }
        

    public static class ElectricalLayout {
        // Controllers
        public final static int CONTROLLER_DRIVER_ID = 0;
        public final static int CONTROLLER_OPERATOR_ID = 1;

        // Drivetrain Test Bot
        /* public final static int DRIVE_FRONT_LEFT = 17;
        public final static int DRIVE_FRONT_RIGHT = 10;
        public final static int DRIVE_BACK_LEFT = 9;
        public final static int DRIVE_BACK_RIGHT = 11; */

        // Drivetrain Main
        public final static int DRIVE_FRONT_LEFT = 1;
        public final static int DRIVE_FRONT_RIGHT = 2;
        public final static int DRIVE_BACK_LEFT = 3;
        public final static int DRIVE_BACK_RIGHT = 4;

        // Intakes
        public final static int INTAKE_MOTOR_LEFT_ID = 12;
        public final static int INTAKE_MOTOR_RIGHT_ID = 13;
        
        public static final int INTAKE_ARM_MOTOR_LEFT_ID = 6;

        // Claw
        public static int CLAW_MOTOR_LEFT_ID = 10;

        // New Elevator Motor Design
        public final static int ELEVATOR_MOTOR_ID = 5;

        // Pivot ARm
        public static int PIVOT_ARM_ID = 7;

        // Sensors
        public static final int INTAKE_BUMP_SWITCH_ID = 24;
        public static final int ELEVATOR_LIMIT_SWITCH = 0;
    };

    public static class ElevatorConstants {

        public static final double[] ELEVATOR_PID = new double[] {0.1, 0, 0, 0};

        // TODO Tune
        public static double ELEVATOR_PID_MAX_OUTPUT = 0.7;
        public static double ELEVATOR_REV_TO_POS_FACTOR = 1; 
        public static double ELEVATOR_PID_TOLERANCE = 3;

        // TODO Tune
        public static final double ELEVATOR_SETPOINT_RETRACT = 0;
        public static final double ELEVATOR_SETPOINT_EXTEND = 32;

        public static final double ELEVATOR_SETPOINT_MIDDLE = (ELEVATOR_SETPOINT_RETRACT + ELEVATOR_SETPOINT_EXTEND) / 2;

        public static final double ELEVATOR_STOP_BUFFER = 5;
    }

    public static class IntakeSpeed {
        public static final double INTAKE_NEUTRAL_SPEED = 0.0;
        public static final double INTAKE_INTAKING_SPEED = 0.85;
        public static final double INTAKE_EJECTING_SPEED = -0.85;
        public static final double INTAKE_SHOOTING_SPEED = 0.95;
    }

    public static class IntakeArmConstants {
        public static final double INTAKE_ARM_PID_TOLERANCE = 0.1;
        public static final double[] INTAKE_ARM_PID = new double[] {0.1, 0, 0.01, 0.01};
        public static final double INTAKE_ARM_PID_MAX_OUTPUT = 0.5;
        public static final double INTAKE_ARM_GEAR_FACTOR = -12; 
        
        public static double INTAKE_SETPOINT_TOP = 1257; // TODO
        public static double INTAKE_SETPOINT_BOT = -1257; // TODO
    }

    public static class Drivetrain {
        // drivetrain constants
        public static double DRIVE_TRACK_WIDTH_M = 0.86;// 0.66; // m
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
        public static double DRIVE_TRAJ_MAX_VEL = 4.0; // m/s
        public static double DRIVE_TRAJ_MAX_ACC = 0.750; //.75;  // m/s^2
        public static double DRIVE_TRAJ_RAMSETE_B = 2.0; // don't change
        public static double DRIVE_TRAJ_RAMSETE_ZETA = 0.7;


        // aligning
        public static double DRIVE_ALIGN_MAX_VEL = 0.75; // m/s
        public static double DRIVE_ALIGN_MAX_ACC = 0.350; //.75;  // m/s^2


        // linear position PID
        public static double[] DRIVE_DIST_PID = { 3.50, 0.0, 0.0 };
        public static double DRIVE_DIST_ANGLE_P = 0.1;
        public static double DRIVE_DIST_TOLERANCE = 0.01;
        public static double DRIVE_DIST_MAX_OUTPUT = 0.6;

        // angular position PID works for test bot
        public static double[] DRIVE_ANGLE_PID = { 0.055, 0.1, 0.005 };
        public static double DRIVE_ANGLE_TOLERANCE = 0.5;
        public static double DRIVE_ANGLE_MAX_OUTPUT = 0.6;

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

        /* public static Transform3d CAMERA_TO_ROBOT = new Transform3d(); */
        
        // TODO Ask Build Team
        public static Transform3d CAMERA_TO_ROBOT_FRONT = new Transform3d(new Translation3d(0.35, 0.15, Units.feetToMeters(6)), new Rotation3d(0, 0, 0));
        public static Transform3d CAMERA_TO_ROBOT_BACK = new Transform3d(new Translation3d(0.35, -0.15 , 0.1), new Rotation3d(0.0, 180.0, 0.0));
        
        // public static AprilTagFieldLayout aprilTagFieldLayout = new
        // AprilTagFieldLayout(AprilTagFields.kDefaultField.m_resourceFile);
        public static String USB_CAMERA_NAME_FRONT = "Front Camera";
        public static String USB_CAMERA_NAME_BACK = "Back Camera";
    };

    public static class Autonomous {
        // all of these positions have been estimated using PathWeaver
        // TODO calculate true positions

        static double trackWidthAdded = Drivetrain.DRIVE_TRACK_WIDTH_M / 2;
        public static double BALANCE_KP = 0.0295;
        public static double BALANCE_KD = 0;
        public static double BALANCE_SETPOINT_ANGLE = 0;
        public static double BALANCE_THRESHOLD_DEGREES = 3;
        public static int BALANCE_STEPS_THRESHOLD = 100;

        public static Pose2d[] BLUE_SCORE_POSE = new Pose2d[] {
                new Pose2d(1.425 + trackWidthAdded, 0.453, Rotation2d.fromDegrees(180)), // Score location 1 on blue side
                new Pose2d(1.425 + trackWidthAdded, 1.044, Rotation2d.fromDegrees(180)), // score 2
                new Pose2d(1.425 + trackWidthAdded, 1.579, Rotation2d.fromDegrees(180)), // 3
                new Pose2d(1.425 + trackWidthAdded, 2.204, Rotation2d.fromDegrees(180)), // ...
                new Pose2d(1.425 + trackWidthAdded, 2.773, Rotation2d.fromDegrees(180)),
                new Pose2d(1.425 + trackWidthAdded, 3.273, Rotation2d.fromDegrees(180)),
                new Pose2d(1.425 + trackWidthAdded, 3.831, Rotation2d.fromDegrees(180)), // todo change
                new Pose2d(1.425 + trackWidthAdded, 4.433, Rotation2d.fromDegrees(180)),
                new Pose2d(1.425 + trackWidthAdded, 5.082, Rotation2d.fromDegrees(180))
        };

        public static Pose2d[] RED_SCORE_POSE = new Pose2d[] {
                new Pose2d(15.15 - trackWidthAdded, 0.453, Rotation2d.fromDegrees(0)), // Score location 1 on blue side
                new Pose2d(15.15 - trackWidthAdded, 1.044, Rotation2d.fromDegrees(0)), // score 2
                new Pose2d(15.15 - trackWidthAdded, 1.579, Rotation2d.fromDegrees(0)), // 3
                new Pose2d(15.15 - trackWidthAdded, 2.204, Rotation2d.fromDegrees(0)), // ...
                new Pose2d(15.15 - trackWidthAdded, 2.773, Rotation2d.fromDegrees(0)),
                new Pose2d(15.15 - trackWidthAdded, 3.273, Rotation2d.fromDegrees(0)),
                new Pose2d(15.15 - trackWidthAdded, 3.831, Rotation2d.fromDegrees(0)),
                new Pose2d(15.15 - trackWidthAdded, 4.433, Rotation2d.fromDegrees(0)),
                new Pose2d(15.15 - trackWidthAdded, 5.082, Rotation2d.fromDegrees(0))
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
                new Pose2d(13.5, 4.73, Rotation2d.fromDegrees(180)),
                new Pose2d(11.6, 4.73, Rotation2d.fromDegrees(180)),
                new Pose2d(13.5, 0.754, Rotation2d.fromDegrees(180)),
                new Pose2d(11.6, 0.754, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d BLUE_CHARGE_POSE[] = new Pose2d[] {
                new Pose2d(3.89, 2.75, Rotation2d.fromDegrees(180)),
                new Pose2d(3.89, 2.75, Rotation2d.fromDegrees(0)),
        };
        public static Pose2d[] RED_CHARGE_POSE = new Pose2d[] {
                new Pose2d(12.58, 2.75, Rotation2d.fromDegrees(0)),
                new Pose2d(12.58, 2.75, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d[] BLUE_CHARGE_POSE_WAYPOINT =new Pose2d[] {
                new Pose2d(5.8, 2.75, Rotation2d.fromDegrees(180)),
                new Pose2d(3.2, 2.75, Rotation2d.fromDegrees(0)),
        };
        public static Pose2d[] RED_CHARGE_POSE_WAYPOINT = new Pose2d[] {
                new Pose2d(10.5, 2.75, Rotation2d.fromDegrees(0)), 
                new Pose2d(13.5, 2.75, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d[] BLUE_LEAVE_COMMUNITY_POSE = new Pose2d[] {
                new Pose2d(5, 4.73, Rotation2d.fromDegrees(0)),
                new Pose2d(5, 0.754, Rotation2d.fromDegrees(0)),
                // this y coord is aligned correctly i think, it's a little jank
                new Pose2d(5.5, BLUE_SCORE_POSE[4].getY(), Rotation2d.fromDegrees(0)),
        };

        public static Pose2d[] RED_LEAVE_COMMUNITY_POSE = new Pose2d[] {
                new Pose2d(11.4, 4.73, Rotation2d.fromDegrees(180)),
                new Pose2d(11.4, 0.754, Rotation2d.fromDegrees(180)),
                new Pose2d(10.7, RED_SCORE_POSE[4].getY(), Rotation2d.fromDegrees(180)),
        };

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

        public static Pose2d BlueNormalEnd = new Pose2d(7.2, 6.251, Rotation2d.fromDegrees(0));
        public static Pose2d RedNormalEnd = new Pose2d(9.4, 6.251, Rotation2d.fromDegrees(180));
        public static Pose2d[] BLUE_PARK_POSE = new Pose2d[] {
                new Pose2d(2.324, 4.593, Rotation2d.fromDegrees(0)),
                new Pose2d(4, 0.716, Rotation2d.fromDegrees(0)),
        };

        public static Pose2d[] RED_PARK_POSE = new Pose2d[] {
                new Pose2d(13.847, 4.593, Rotation2d.fromDegrees(180)),
                new Pose2d(12.57, 0.716, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d[] Straightline = new Pose2d[] {
                new Pose2d(23.034, 17.493, Rotation2d.fromDegrees(0)),
                new Pose2d(41.914, 17.493, Rotation2d.fromDegrees(0)),
        };

        public static Pose2d[] BLUE_SHOOT_POSE = new Pose2d[] {
                new Pose2d(2.324, 4.593, Rotation2d.fromDegrees(180)),
                new Pose2d(4, 0.716, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d[] RED_SHOOT_POSE = new Pose2d[] {
                new Pose2d(13.847, 4.593, Rotation2d.fromDegrees(0)),
                new Pose2d(12.57, 0.716, Rotation2d.fromDegrees(0)),
        };


        public static double CHARGE_STATION_LOWER_Y = 1.508506;
        public static double CHARGE_STATION_UPPER_Y = 3.978656;

        public static final double BLUE_COMMUNITY_X = 3.34;
        public static final double RED_COMMUNITY_X = 13.2;

        public static double CHARGE_CENTER_Y = 2.75;

    };

    public static class Claw {
        /* public static double[] CLAW_PID = new double[] {0, 0, 0, 0};
        public static double CLAW_PID_TOLERANCE = 0.1257; // lol
        public static double CLAW_PID_MAX_OUTPUT = 0.1257; // lol */

        public static double CLAW_CLOSED_SPEED = 0.5;
        public static double CLAW_OPEN_SPEED = -0.25;

        public static double POSITION_CONVERSION_FACTOR = 1;

        public static double CLAW_OPEN_TIME = 2.0;
        public static double CLAW_CLOSE_TIME = 2.20;
    };

    public static class PivotArm {
        // PID constants
        public static double[] PIVOT_ARM_PID = new double[] {0.25, 0, 0, 0};
        public static double PIVOT_ARM_PID_TOLERANCE = 1; 
        public static double PIVOT_ARM_PID_MAX_OUTPUT = 0.8; 

        public static double POSITION_CONVERSION_FACTOR = 1;

        // Setpoints between -1 and 1
        public static double PIVOT_ARM_SETPOINT_UP = 135;
        public static double PIVOT_ARM_SETPOINT_MID = 175;
        // public static double PIVOT_ARM_SETPOINT_INTAKE = 0; // also used for low score
        public static double PIVOT_ARM_SETPOINT_HOLD = 10;
        public static final double PIVOT_ARM_SETPOINT_BOTTOM = 0;
        public static final double PIVOT_ARM_SETPOINT_TOP = 170;
    }

    public static class LED {
        public static int LED_PORT_NUMBER = 1257;
        public static int LED_COUNT = 1257;
    }

    public static double PI = 3.141592653589793238462643;
    public static double UPDATE_PERIOD = 0.010; // seconds
    public final static int NEO_550_CURRENT_LIMIT = 25; // amps
 
    /** Ambiguous with NEO_CURRENT_LIMIT in ElectricalLayout */
    // public final static int NEO_CURRENT_LIMIT = 80; // amps

    public final static int NEO_CURRENT_LIMIT = 80; // amps
}



