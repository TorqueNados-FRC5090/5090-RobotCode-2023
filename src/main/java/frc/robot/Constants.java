package frc.robot;
// Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/* 
 * To access numbers in this file, import or statically import one of its subclasses:
 * example:
 * import static frc.robot.Constants.ControllerPorts.*;
 * import frc.robot.Constants.DriveConstants;
 */
public final class Constants {

    /* -------------- IDs -------------- */

    /** Ports used by controllers. */
    public static final class ControllerPorts {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    /** IDs used by the drivetrain.
        3X for turning, 4X for driving, 5X for abs encoders. */
    public static final class SwerveIDs {
        // Front left module
        public static final int FL_DRIVE_ID = 40;
        public static final int FL_TURN_ID = 30;
        public static final int FL_ENCODER_ID = 50;
        // Front right module
        public static final int FR_DRIVE_ID = 41;
        public static final int FR_TURN_ID = 31;
        public static final int FR_ENCODER_ID = 51;
        // Rear left module
        public static final int RL_DRIVE_ID = 42;
        public static final int RL_TURN_ID = 32;
        public static final int RL_ENCODER_ID = 52;
        // Rear right module
        public static final int RR_DRIVE_ID = 43;
        public static final int RR_TURN_ID = 33;
        public static final int RR_ENCODER_ID = 53;
    }

    /** IDs used by the Arm */
    public static final class ArmIDs {
        public static final int ROTATION_ID = 10;
        public static final int TELESCOPE_ID = 11;
        public static final int TELESCOPE_FOLLOWER_ID = 12;
        public static final int SLIDER_ID = 13;
    }

    /* -------------- SUBSYTEM CONSTANTS -------------- */

    /** Contains the ratios for the arm */
    public static final class ArmConstants {
        /** Desired movement in inches * TELESCOPE_RATIO = required revolutions of the motor */
        public static final double TELESCOPE_RATIO = 8.3808246;
        /** Desired movement in inches * SLIDER_RATIO = required revolutions of the motor */
        public static final double SLIDER_RATIO = 4.33650762;
        /** Desired movement in degrees * ROTATION_RATIO = required revolutions of the motor */
        public static final double ROTATION_RATIO = .277777777;
    }

    /** Constants used by the limelight */
    public static final class LimelightConstants {
        // LIMELIGHT DATA IS OUT OF DATE

        // Limelight calculation numbers
        public static final int LIME_HEIGHT = 39; // Height of limelight off the ground in inches
        public static final int TARGET_HEIGHT = 104; // Height of target in inches
        public static final double LIME_ANGLE = Math.toRadians(51); // Angle of limelight relative to the floor

        // Limelight tuning numbers
        public static final double DESIRED_TARGET_AREA = 1.5;  // Area of the target when the robot reaches the wall
        public static final double DRIVE_K = 0.36;             // How fast to drive toward the target
        public static final double STEER_K = 0.05;             // How quickly the robot turns toward the target
        public static final double MAX_DRIVE = 0.5;            // Simple speed limit so we don't drive too fast
    }

    /** Turning a module to its offset will point it forward */
    public static final class SwerveModuleOffsets {
        public static final double FL_OFFSET = 120;
        public static final double FR_OFFSET = 188;
        public static final double RL_OFFSET = 253;
        public static final double RR_OFFSET = 135;
    }

    /** Whether or not each swerve component should be inverted/reversed */
    public static final class SwerveInversions {
        // Whether each driving motor should be inverted
        public static final boolean INVERT_FL_DRIVE = false;
        public static final boolean INVERT_RL_DRIVE = false;
        public static final boolean INVERT_FR_DRIVE = true;
        public static final boolean INVERT_RR_DRIVE = true;

        // Whether each turning motor should be inverted
        public static final boolean INVERT_FL_TURN = true;
        public static final boolean INVERT_FR_TURN = true;
        public static final boolean INVERT_RL_TURN = true;
        public static final boolean INVERT_RR_TURN = true;
    }

    /** Constants related to swerve calculations */
    public static final class SwerveConstants {
        /** The distance between the left and right wheels */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        /** The distance between the front and rear wheels */
        public static final double WHEEL_BASE = Units.inchesToMeters(27);

        /** An array containing the position of each module as a {@link Translation2d} object */
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };

        // Kinematics are used to calculate how each module needs to move
        // in order to move the robot as a whole in a certain way

        /** Standard kinematics with center of rotation located at the center of the robot */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        // PID values for the X, Y, and Theta controllers
        public static final double X_CONTROLLER_P = 0.2;
        public static final double X_CONTROLLER_D = 0;
        public static final double Y_CONTROLLER_P = 0.2;
        public static final double Y_CONTROLLER_D = 0;
        public static final double THETA_CONTROLLER_P = 8;
        public static final double THETA_CONTROLLER_D = 0;


        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxRotationRadiansPerSecond = Math.PI;
        public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2;
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

        public static final class ModuleConstants {
            /** The ratio of the drive motors on the workhorse chassis */
            public static final double DRIVE_RATIO_SLOW = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
            /** The ratio of the drive motors on the robot */
            public static final double DRIVE_RATIO_FAST = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
            /** The ratio of the turning motors */
            public static final double TURN_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
            
            /** The diameter of the wheels measured in meters */
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.94);

            /** Drive motor revolutions * DRIVE_REVS_TO_M = distance in meters */
            public static final double DRIVE_REVS_TO_M =
                ((WHEEL_DIAMETER * Math.PI) / DRIVE_RATIO_FAST); // Uses fast ratio

            // 1 RPM * DRIVE_REVS_TO_M = speed in m/min. Divide by 60 to find m/sec
            /** Drive motor RPM * DRIVE_RPM_TO_MPS = speed in m/sec */
            public static final double DRIVE_RPM_TO_MPS =
                DRIVE_REVS_TO_M / 60.0;
            public static final double kTurnRotationsToDegrees =
                360.0 / TURN_RATIO;

            public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
            public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
            public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;
        }
    
        /** Enum representing the four possible positions a module can occupy */
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
    }

  public static final class DriveConstants {
            /** Higher values make the robot drive more aggressively */
            public static final double TRANSLATION_SLEW = 4;
            /** Higher values make the robot spin more aggressively */
            public static final double ROTATION_SLEW = 6;
    
            /** Translation instructions closer to 0 than the deadband will be set to 0 */
            public static final double TRANSLATION_DEADBAND = .05;
            /** Rotation instructions closer to 0 than the deadband will be set to 0 */
            public static final double ROTATION_DEADBAND = .1;
    }
}
