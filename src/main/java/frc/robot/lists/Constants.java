package frc.robot.lists;

// Imports
import java.util.Map;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;


public final class Constants {

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
    public static final class ModuleOffsets {
        public static final double FL_OFFSET = 120;
        public static final double FR_OFFSET = 188;
        public static final double RL_OFFSET = 253;
        public static final double RR_OFFSET = 135;
    }

    /** Whether or not each swerve component should be inverted/reversed */
    public static final class SwerveInversions {
        public static final boolean INVERT_FL_TURN = true;
        public static final boolean INVERT_FR_TURN = true;
        public static final boolean INVERT_RL_TURN = true;
        public static final boolean INVERT_RR_TURN = true;

        public static final boolean INVERT_FL_DRIVE = false;
        public static final boolean INVERT_RL_DRIVE = false;
        public static final boolean INVERT_FR_DRIVE = true;
        public static final boolean INVERT_RR_DRIVE = true;

        public static final boolean INVERT_GYRO = true;
    }

    /** Constants related to the position of each module */
    public static final class ModulePositions {
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(27);


        /** The position of each module in writing as an enum */
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }

        /** The position of each module stored as a coordinate */
        public static final Map<ModulePosition, Translation2d> MODULE_POSITIONS = Map.of(
            ModulePosition.FRONT_LEFT, new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ModulePosition.FRONT_RIGHT, new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ModulePosition.REAR_LEFT, new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ModulePosition.REAR_RIGHT, new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
            
        /** {@link SwerveDriveKinematics} */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            ModuleMap.orderedValues(MODULE_POSITIONS, new Translation2d[0]));
    }
    
    /** Constants used to drive the robot */
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final class DriveConstants {
        public static final double STATIC_GAIN = 1;
        public static final double VELOCITY_GAIN = 0.8;
        public static final double ACCELERATION_GAIN = 0.15;

        /** Max speed of the robot in m/sec */
        public static final double MAX_TRANSLATION_SPEED = 3;
        /** Max rotational speed of the robot in rads/sec */
        public static final double MAX_ROTATION_SPEED = Math.PI;
        public static final double MAX_ROTATION_SPEED_SQUARED = Math.PI;

        public static final double X_CONTROLLER_P = 0.2;
        public static final double X_CONTROLLER_D = 0;
        public static final double Y_CONTROLLER_P = 0.2;
        public static final double Y_CONTROLLER_D = 0;
        public static final double TURN_CONTROLLER_P = 8;
        public static final double TURN_CONTROLLER_D = 0;

        /** Higher values make the robot drive more aggressively */
        public static double TRANSLATION_SLEW = 4;
        /** Higher values make the robot spin more aggressively */
        public static double ROTATION_SLEW = 6;

        /** Translation instructions closer to 0 than the deadband will be set to 0 */
        public static double TRANSLATION_DEADBAND = .05;
        /** Rotation instructions closer to 0 than the deadband will be set to 0 */
        public static double ROTATION_DEADBAND = .1;
        
        public static double VOLT_COMPENSATION = 12.6;
    }

    /** Constants used by individual modules */
    public static final class ModuleConstants {
        /** The diameter of a module's wheel, measured in meters */
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        // Swerve gear ratios
        public static double DRIVE_RATIO_SLOW = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
        public static double DRIVE_RATIO_FAST = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
        public static double TURN_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));

        /** Drive motor revolutions * DRIVE_REVS_TO_M = distance in meters */
        public static final double DRIVE_REVS_TO_M = (WHEEL_DIAMETER * Math.PI) / DRIVE_RATIO_SLOW;

        // 1 RPM * DRIVE_REVS_TO_M = speed in m/min. Divide by 60 to find m/sec
        /** Drive motor RPM * DRIVE_RPM_TO_MPS = speed in m/sec */
        public static final double DRIVE_RPM_TO_MPS = DRIVE_REVS_TO_M / 60;// 0.000653304296852

        public static double kEncoderRevsPerMeter = 1 / DRIVE_REVS_TO_M;// 25.511337897182322

        public static double kFreeMetersPerSecond = 5600 * DRIVE_RPM_TO_MPS;// 3.6
    
        public static final double kTurningDegreesPerEncRev = 360 / TURN_RATIO;

        // Use sysid on robot to get these values
        public static double STATIC_GAIN = .055;
        public static double VELOCITY_GAIN = .2;
        public static double ACCELERATION_GAIN = .02;

        // Used, but uninitialized and uninstantiated
        public static double kPModuleTurnController;


        // ------ Values below this line are unused ------

        // max turn speed = (5400/ 21.43) revs per min 240 revs per min 4250 deg per min
        public static final double kPModuleTurningController = .025;

        public static final double kPModuleDriveController = .2;

        public static double kSMmaxAccel = 90;//deg per sec per sec

        public static double maxVel = 90; // deg per sec

        public static double allowedErr = .05;//deg

        // sysid on module?
        public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
        public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
        public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;
        // sysid on module?
        public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
        public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
        
        public static double kMaxModuleAngularSpeedDegPerSec = 90;

        public static final double kMaxModuleAngularAccelerationDegreesPerSecondSquared = 90;
    }

    /** Constants related to the trapezoid constraint profile */
    public static final class TrapezoidConstants {
        // Unused values
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;


        public static final double MAX_WHEEL_SPEED = Math.PI;
        public static final double MAX_WHEEL_SPEED_SQUARED = Math.PI * 2;

        public static final TrapezoidProfile.Constraints TURN_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_WHEEL_SPEED, MAX_WHEEL_SPEED_SQUARED);
    }
}
