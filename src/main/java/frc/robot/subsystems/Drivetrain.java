// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveIDs;
import frc.robot.Constants.SwerveInversions;
import frc.robot.Constants.SwerveModuleOffsets;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.SwerveConstants.ModulePosition;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.SwerveConstants.*;


public class Drivetrain extends SubsystemBase {

    private SwerveModule frontLeftModule = new SwerveModule( 0,
        new CANSparkMax(SwerveIDs.FL_DRIVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANSparkMax(SwerveIDs.FL_TURN_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANCoder(SwerveIDs.FL_ENCODER_ID),
        SwerveInversions.INVERT_FL_DRIVE,
        SwerveInversions.INVERT_FL_TURN,
        SwerveModuleOffsets.FL_OFFSET);

    private SwerveModule frontRightModule = new SwerveModule( 1,
        new CANSparkMax(SwerveIDs.FR_DRIVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANSparkMax(SwerveIDs.FR_TURN_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANCoder(SwerveIDs.FR_ENCODER_ID),
        SwerveInversions.INVERT_FR_DRIVE,
        SwerveInversions.INVERT_FR_TURN,
        SwerveModuleOffsets.FR_OFFSET);

    private SwerveModule rearLeftModule = new SwerveModule( 2,
        new CANSparkMax(SwerveIDs.RL_DRIVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANSparkMax(SwerveIDs.RL_TURN_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANCoder(SwerveIDs.RL_ENCODER_ID),
        SwerveInversions.INVERT_RL_DRIVE,
        SwerveInversions.INVERT_RL_TURN,
        SwerveModuleOffsets.RL_OFFSET);

    private SwerveModule rearRightModule = new SwerveModule( 3,
        new CANSparkMax(SwerveIDs.RR_DRIVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANSparkMax(SwerveIDs.RR_TURN_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
        new CANCoder(SwerveIDs.RR_ENCODER_ID),
        SwerveInversions.INVERT_RR_DRIVE,
        SwerveInversions.INVERT_RR_TURN,
        SwerveModuleOffsets.RR_OFFSET);

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

    // TODO add a useful comment here
    private SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            SwerveConstants.SWERVE_KINEMATICS,
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());

    /* Will potentially be used to track robot movement in auton
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


        translationY *= kMaxSpeedMetersPerSecond;
        translationX *= kMaxSpeedMetersPerSecond;
        rotation *= kMaxRotationRadiansPerSecond;

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
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);

        // Send instructions to each module
        for (SwerveModule module : swerveModules.values())
            module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
    }


    /** @return The current direction the robot is facing in degrees */
    public double getHeadingDegrees() { return Math.IEEEremainder(gyro.getYaw(), 360); }
    /** @return The current direction the robot is facing as a {@link Rotation2d} object */
    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    /** @return The position in meters and direction of the robot in degrees as a {@link Pose2d} object */
    public Pose2d getPoseMeters() { return odometry.getPoseMeters(); }
    /** @param moduleNumber The index of the module 
     *  @return A single {@link SwerveModule swerve module} */
    public SwerveModule getSwerveModule(int moduleNumber) { return swerveModules.get(ModulePosition.values()[moduleNumber]); }
    /** @param position The {@link ModulePosition position} of the module
     *  @return A single {@link SwerveModule swerve module} */
    public SwerveModule getSwerveModule(ModulePosition position) { return swerveModules.get(ModulePosition.FRONT_LEFT); }
    
    // Methods related to field orientation
    /** @return Whether or not the robot is in field oriented mode */
    public boolean isFieldCentric() { return isFieldCentric; }
    /** @return The opposite of isFieldCentric() */
    public boolean isRobotCentric() { return !isFieldCentric; }
    /** @param isFieldCentric Whether the robot should be set to field centric or not */
    public void setFieldCentric(boolean isFieldCentric) { this.isFieldCentric = isFieldCentric; }
    /** @param isFieldCentric Whether the robot should be set to robot centric or not */
    public void setRobotCentric(boolean isFieldCentric) { this.isFieldCentric = !isFieldCentric; }
    /** Sets the robot to field centric if currently robot centric and vice versa */
    public void toggleFieldCentric() { isFieldCentric = !isFieldCentric; }

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
     *  of each module and the current heading of the robot
     */
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

    
    @Override // Called every 20ms
    public void periodic() {
        updateOdometry();
    }
}