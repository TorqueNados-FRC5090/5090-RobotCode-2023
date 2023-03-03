package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Math Imports
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import java.util.Map;
import java.util.HashMap;

// Gyro imports
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

// Import constants
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.Constants.SwerveConstants;
import static frc.robot.Constants.SwerveIDs.*;
import static frc.robot.Constants.SwerveInversions.*;
import static frc.robot.Constants.SwerveModuleOffsets.*;
import static frc.robot.Constants.SwerveConstants.*;

/** This class represents the drivetrain on the robot */
public class Drivetrain extends SubsystemBase {

    // Construct each swerve module
    /** The front left (FL) {@link SwerveModule}. Module number is 0 */
    private SwerveModule frontLeftModule = new SwerveModule(0,
        FL_DRIVE_ID,
        FL_TURN_ID,
        FL_ENCODER_ID,
        INVERT_FL_DRIVE,
        INVERT_FL_TURN,
        FL_OFFSET);

    /** The front right (FR) {@link SwerveModule}. Module number is 1 */
    private SwerveModule frontRightModule = new SwerveModule(1,
        FR_DRIVE_ID,
        FR_TURN_ID,
        FR_ENCODER_ID,
        INVERT_FR_DRIVE,
        INVERT_FR_TURN,
        FR_OFFSET);

    /** The rear left (RL) {@link SwerveModule}. Module number is 2 */
    private SwerveModule rearLeftModule = new SwerveModule(2,
        RL_DRIVE_ID,
        RL_TURN_ID,
        RL_ENCODER_ID,
        INVERT_RL_DRIVE,
        INVERT_RL_TURN,
        RL_OFFSET);

    /** The rear right (RR) {@link SwerveModule}. Module number is 3 */
    private SwerveModule rearRightModule = new SwerveModule(3,
        RR_DRIVE_ID,
        RR_TURN_ID,
        RR_ENCODER_ID,
        INVERT_RR_DRIVE,
        INVERT_RR_TURN,
        RR_OFFSET);

    /** A {@link HashMap} associating each {@link SwerveModule module} with its {@link ModulePosition position} */
    private final HashMap<ModulePosition, SwerveModule> swerveModules =
        new HashMap<>(
            Map.of(
                ModulePosition.FRONT_LEFT,
                frontLeftModule,

                ModulePosition.FRONT_RIGHT,
                frontRightModule,

                ModulePosition.REAR_LEFT,
                rearLeftModule,

                ModulePosition.REAR_RIGHT,
                rearRightModule));

    /** The gyro is used to help keep track of where the robot is facing */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    /** While the robot is in field centric mode, forward is a defined direction.
     *  Conversely, if the robot is not in field centric mode, it is robot centric.
     *  While the robot is in robot centric mode, forward is whichever direction the robot is facing. */
    private boolean isFieldCentric = true;

    /** Used to track the robot's position as it moves */
    private SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            SwerveConstants.SWERVE_KINEMATICS,
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());

    /* These will potentially be used to track robot movement in auton
    private ProfiledPIDController xController =
        new ProfiledPIDController(kP_X, 0, kD_X, kThetaControllerConstraints);
    private ProfiledPIDController yController =
        new ProfiledPIDController(kP_Y, 0, kD_Y, kThetaControllerConstraints);
    private ProfiledPIDController turnController =
        new ProfiledPIDController(kP_Theta, 0, kD_Theta, kThetaControllerConstraints);
    */
    
    /** Constructs a drivetrain {@link SubsystemBase subsystem} */
    public Drivetrain() {
        gyro.reset();
    }

    /** 
     * Drives the robot
     *  
     * @param translationX The left/right translation instruction
     * @param translationY The forward/back translation instruction
     * @param rotation The rotational instruction
    */
    public void drive( double translationX, double translationY, double rotation, boolean isOpenLoop ) {


        translationY *= MAX_TRANSLATION_SPEED;
        translationX *= MAX_TRANSLATION_SPEED;
        rotation *= MAX_ROTATION_SPEED;

        ChassisSpeeds chassisSpeeds =
            isFieldCentric
                // Calculate field relative instructions if isFieldCentric is true
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translationY, translationX, rotation, getHeadingRotation2d())
                // Calculate robot centric instructions if isFieldCentric is false
                : new ChassisSpeeds(translationY, translationX, rotation);

        // Convert ChassisSpeed instructions to useable SwerveModuleStates
        SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Normalize output if any of the modules would be instructed to go faster than possible
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_TRANSLATION_SPEED);

        // Send instructions to each module
        for (SwerveModule module : swerveModules.values())
            module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
    }


    /** @return The current direction the robot is facing in degrees */
    public double getHeadingDegrees() { return -Math.IEEEremainder(gyro.getAngle(), 360); }
    /** @return The current direction the robot is facing as a {@link Rotation2d} object */
    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    /** Reset the heading of the robot, effectively changing the orientation of the field */
    public void resetHeading() { gyro.reset(); }
    /** @return The position in meters and direction of the robot in degrees as a {@link Pose2d} object */
    public Pose2d getPoseMeters() { return odometry.getPoseMeters(); }
    /** @param moduleNumber The index of the module 
     *  @return The {@link SwerveModule swerve module} at that index */
    public SwerveModule getSwerveModule(int moduleNumber) { return swerveModules.get(ModulePosition.values()[moduleNumber]); }
    /** @param position The {@link ModulePosition position} of the module
     *  @return The {@link SwerveModule swerve module} at that position */
    public SwerveModule getSwerveModule(ModulePosition position) { return swerveModules.get(ModulePosition.FRONT_LEFT); }
    
    // Methods related to field orientation
    /** @return Whether or not the robot is in field oriented mode */
    public boolean isFieldCentric() { return isFieldCentric; }
    /** @return The opposite of isFieldCentric() */
    public boolean isRobotCentric() { return !isFieldCentric; }
    /** @param isFieldCentric Whether the robot should be set to field centric or not */
    public void setFieldCentric(boolean isFieldCentric) { this.isFieldCentric = isFieldCentric; }
    /** @param isFieldCentric Whether the robot should be set to robot centric or not */
    public void setRobotCentric(boolean isRobotCentric) { this.isFieldCentric = !isFieldCentric; }
    /** Sets the robot to field centric if currently robot centric and vice versa */
    public void toggleFieldCentric() { isFieldCentric = !isFieldCentric; }

    /** Sets the wheels of the robot into an X shape for anti-defense */
    public void lock() {
        int[] lockPos = {-45,45,-45,45};
        int i = 0;

        for (SwerveModule module : swerveModules.values()){
            module.setDesiredState(
            new SwerveModuleState(
                0, 
                new Rotation2d(lockPos[i])),    
            true);
            i++;
        }
    }

    public void zeroWheels() { 
         for (SwerveModule module : swerveModules.values())
            module.setDesiredState(
            new SwerveModuleState(
                0, 
                new Rotation2d(0)),    
            true);
    }

    /** @return An array containing the current {@link SwerveModuleState state} of each module */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
            swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
            swerveModules.get(ModulePosition.REAR_LEFT).getState(),
            swerveModules.get(ModulePosition.REAR_RIGHT).getState()
        };
    }
    /** @return An array containing the current {@link SwerveModulePosition position} of each module */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
            swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
            swerveModules.get(ModulePosition.REAR_LEFT).getPosition(),
            swerveModules.get(ModulePosition.REAR_RIGHT).getPosition()
        };
    }

    /** Updates the odometry of the robot using the {@link SwerveModulePosition position} 
     *  of each module and the current heading of the robot */
    public void updateOdometry() {
        odometry.update(getHeadingRotation2d(), getModulePositions());

        for (SwerveModule module : swerveModules.values()) {
        var modulePositionFromChassis =
            MODULE_TRANSLATIONS[module.getModuleNumber()]
                .rotateBy(getHeadingRotation2d())
                .plus(getPoseMeters().getTranslation());
        module.setModulePose(
            new Pose2d(
                modulePositionFromChassis,
                module.getHeadingRotation2d().plus(getHeadingRotation2d())));
        }
    }

     /**resets the odometry of the modules */
     public void resetOdometry(){
        odometry.resetPosition(
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());
    }


    
    @Override // Called every 20ms
    public void periodic() {
        updateOdometry();
    }
}