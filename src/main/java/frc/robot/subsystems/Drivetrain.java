package frc.robot.subsystems;

// Map imports
import java.util.HashMap;
import java.util.Map;
import frc.robot.utils.ModuleMap;

// Math and swerve imports
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Import constants
import frc.robot.lists.Constants;
import frc.robot.lists.Constants.CanConstants;
import frc.robot.lists.Constants.DriveConstants;
import frc.robot.lists.Constants.DriveConstants.ModulePosition;

// Other imports
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ShuffleboardContent;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.unmanaged.Unmanaged;

// This subsystem represents the robot's drivetrain.
// In this case, a drivetrain consists of four swerve modules arranged in a square.
public class Drivetrain extends SubsystemBase {

    public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

    // Initialize all four motors in a hashmap
    public final HashMap<ModulePosition, SwerveModule> swerveModules = new HashMap<>(
        Map.of(
            ModulePosition.FRONT_LEFT,
            new SwerveModule(ModulePosition.FRONT_LEFT,
                CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
                DriveConstants.kFrontLeftDriveMotorReversed,
                DriveConstants.kFrontLeftTurningMotorReversed,
                CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET),

            ModulePosition.FRONT_RIGHT,
            new SwerveModule(
                ModulePosition.FRONT_RIGHT,
                CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
                DriveConstants.kFrontRightDriveMotorReversed,
                DriveConstants.kFrontRightTurningMotorReversed,
                CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET),

            ModulePosition.BACK_LEFT,
            new SwerveModule(ModulePosition.BACK_LEFT,
                CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
                DriveConstants.kBackLeftDriveMotorReversed,
                DriveConstants.kBackLeftTurningMotorReversed,
                CanConstants.BACK_LEFT_MODULE_STEER_OFFSET),

            ModulePosition.BACK_RIGHT,
            new SwerveModule(
                ModulePosition.BACK_RIGHT,
                CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
                DriveConstants.kBackRightDriveMotorReversed,
                DriveConstants.kBackRightTurningMotorReversed,
                CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET)));

    // The gyro sensor
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    // Used for driving autonomously
    private PIDController xController = new PIDController(DriveConstants.kP_X, 0, DriveConstants.kD_X);
    private PIDController yController = new PIDController(DriveConstants.kP_Y, 0, DriveConstants.kD_Y);
    private ProfiledPIDController turnController = new ProfiledPIDController(
        DriveConstants.kP_Theta, 0,
        DriveConstants.kD_Theta,
        Constants.TrapezoidConstants.kThetaControllerConstraints);

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
        getHeadingRotation2d(),
        new Pose2d(),
        kSwerveKinematics,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.05),
        VecBuilder.fill(0.1, 0.1, 0.1));

    private SimDouble simAngle;
    public double throttleValue;
    public double targetAngle;
    public boolean fieldOriented;

    // Constructor
    public Drivetrain() {
        // Set default settings
        gyro.reset();
        resetModuleEncoders();
        setIdleMode(true); // Brakes active in idle
        fieldOriented = false; // Start in robot-centric

        if (RobotBase.isSimulation()) {
            var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
        }

        ShuffleboardContent.initMisc(this);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param throttle Speed of the robot in the x direction (forward).
     * @param strafe Speed of the robot in the y direction (sideways).
     * @param rotation Angular rate of the robot.
     * @param isOpenLoop Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double throttle, double strafe, double rotation, boolean isOpenLoop) {
        throttle *= DriveConstants.kMaxSpeedMetersPerSecond;
        strafe *= DriveConstants.kMaxSpeedMetersPerSecond;
        rotation *= DriveConstants.kMaxRotationRadiansPerSecond;

        SmartDashboard.putNumber("Rotn1", rotation);
        ChassisSpeeds chassisSpeeds =fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(throttle, strafe, rotation);

        Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
            .of(kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

        SwerveDriveKinematics.desaturateWheelSpeeds(
            ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), DriveConstants.kMaxSpeedMetersPerSecond);

        for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules))
            module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
    }

    // Getters
    public PIDController getXPidController() { return xController; }
    public PIDController getYPidController() { return yController; }
    public ProfiledPIDController getThetaPidController() { return turnController; }
    public SwerveModule getSwerveModule(ModulePosition modulePosition) { return swerveModules.get(modulePosition); }
    public SwerveDrivePoseEstimator getOdometry() { return odometry; }
    public Pose2d getPoseMeters() { return odometry.getEstimatedPosition(); }
    public Translation2d getTranslation() { return getPoseMeters().getTranslation(); }
    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    public double getHeadingDegrees() { return -Math.IEEEremainder((gyro.getAngle()), 360); }
    public double getAnglefromThrottle() { return 180 * throttleValue; }
    public double getX() { return getTranslation().getX(); }
    public double getY() { return getTranslation().getY(); }
    public boolean getTurnInPosition(ModulePosition mp, double targetAngle) { return getSwerveModule(mp).turnInPosition(targetAngle); }

    // Setters
    public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
        
        for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules))
        module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
    }
    public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
        setSwerveModuleStates(states, false);
    }
    public void setOdometry(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
        gyro.reset();
    }
    public void setIdleMode(boolean brake) {
        for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules)) {
            module.setDriveBrakeMode(brake);
            module.setTurnBrakeMode(brake);
        }
    }

    public Map<ModulePosition, SwerveModuleState> getModuleStates() {
        Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
        for (ModulePosition i : swerveModules.keySet()) {
        map.put(i, swerveModules.get(i).getState());
        }
        return map;
    }

    public void resetModuleEncoders() {
        for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules))
        module.resetAngleToAbsolute();
    }

    public void zeroHeading() {
        // gyro.reset();
        // gyro.setAngleAdjustment(0);
    }

    public double reduceRes(double value, int numPlaces) {
        double n = Math.pow(10, numPlaces);
        return Math.round(value * n) / n;
    }

    // Turn a single module at a % speed
    public void turnModule(ModulePosition mp, double speed) {
        getSwerveModule(mp).turnMotorMove(speed);
    }

    // Turn a single module to a position
    public void positionTurnModule(ModulePosition mp, double angle) {
        getSwerveModule(mp).positionTurn(angle);
    }

    // Drive a single module at a % speed
    public void driveModule(ModulePosition mp, double speed) {
        getSwerveModule(mp).driveMotorMove(speed);
    }

    public void updateOdometry() {
        odometry.update(
            getHeadingRotation2d(),
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

        for (SwerveModule module : ModuleMap.orderedValuesList(swerveModules)) {
        Translation2d modulePositionFromChassis = DriveConstants.kModuleTranslations
            .get(module.getModulePosition())
            .rotateBy(getHeadingRotation2d())
            .plus(getPoseMeters().getTranslation());
        module.setModulePose(
            new Pose2d(
                modulePositionFromChassis,
                module.getHeadingRotation2d().plus(getHeadingRotation2d())));
        }
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();
        SmartDashboard.putNumber("Yaw",-gyro.getYaw());
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds chassisSpeedSim = kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
        
        // want to simulate navX gyro changing as robot turns
        // information available is radians per second and this happens every 20ms
        // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
        // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
        // degree adder would be radian adder * 360/2pi
        // so degree increment multiplier is 360/100pi = 1.1459
        double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;
        temp += simAngle.get();
        simAngle.set(temp);
        Unmanaged.feedEnable(20);
    }
}