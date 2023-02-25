// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.SwerveConstants.ModuleConstants.*;
import static frc.robot.Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;

public class SwerveModule extends SubsystemBase {
    private final int VEL_SLOT = 1;

    private int m_moduleNumber;
    private CANSparkMax m_turnMotor;
    private CANSparkMax m_driveMotor;
    private SwerveModuleState state;
    private SparkMaxPIDController m_driveController;
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turnEncoder;
    private PIDController turnController = new PIDController(.007, .00175, .0000625);
    private CANCoder m_angleEncoder;
    private double m_angleOffset;
    private double m_lastAngle;
    private Pose2d m_pose;

    /**
     * Constructs a SwerveModule.
     *
     * @param position The position of the module being constructed. (see enum)
     * @param driveMotor The ID of the driving motor.
     * @param turnMotor The ID of the turning motor.
     * @param absoluteEncoder The ID of the absolute encoder.
     * @param driveMotorInverted Whether the driving motor is inverted.
     * @param turningMotorInverted Whether the turning motor is inverted.
     * @param turningEncoderOffset The encoder's reading when pointing forward.
     */
    public SwerveModule(
        int moduleNumber,
        CANSparkMax driveMotor,
        CANSparkMax turnMotor,
        CANCoder absoluteEncoder,
        boolean driveMotorInverted,
        boolean turningMotorInverted,
        double turningEncoderOffset) {


        m_moduleNumber = moduleNumber;
        m_turnMotor = turnMotor;
        m_driveMotor = driveMotor;
        m_angleEncoder = absoluteEncoder;
        m_angleOffset = turningEncoderOffset;

        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setSmartCurrentLimit(20);
        m_driveMotor.getPIDController().setFF(0.0);
        m_driveMotor.getPIDController().setP(0.2);
        m_driveMotor.getPIDController().setI(0.0);
        m_driveMotor.setInverted(driveMotorInverted);
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        m_driveMotor.enableVoltageCompensation(12.6);
        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.setSmartCurrentLimit(20);
        m_turnMotor.getPIDController().setFF(0.0);
        m_turnMotor.getPIDController().setP(0.2);
        m_turnMotor.getPIDController().setI(0.0);
        m_turnMotor.setInverted(turningMotorInverted);
        m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        m_turnMotor.enableVoltageCompensation(12.6);
        m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPositionConversionFactor(DRIVE_REVS_TO_M);
        m_driveEncoder.setVelocityConversionFactor(DRIVE_RPM_TO_MPS);

        m_turnEncoder = m_turnMotor.getEncoder();
        m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees);
        m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60);

        m_driveController = m_driveMotor.getPIDController();

        resetAngleToAbsolute();
    }

    public int getModuleNumber() {
        return m_moduleNumber;
    }

    public void resetAngleToAbsolute() {
        double angle = m_angleEncoder.getAbsolutePosition() - m_angleOffset;
        m_turnEncoder.setPosition(angle);
    }

    public double getHeadingDegrees() {
        return m_turnEncoder.getPosition();
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    public double getDriveMeters() {
        return m_driveEncoder.getPosition();
    }
    public double getDriveMetersPerSecond() {
        return m_driveEncoder.getVelocity();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        state = RevUtils.optimize(desiredState, getHeadingRotation2d());

        if (isOpenLoop) {
            double percentOutput = state.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
            m_driveMotor.set(percentOutput);
        } 
        else {
            int DRIVE_PID_SLOT = VEL_SLOT;
            m_driveController.setReference(
                state.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                DRIVE_PID_SLOT
            );
        }

        double angle =
            (Math.abs(state.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                ? m_lastAngle
                : state.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    
            positionTurn(angle);
    }

    public void positionTurn(double angle) {
        double turnAngleError = Math.abs(angle - m_turnEncoder.getPosition());

        double pidOut = turnController.calculate(m_turnEncoder.getPosition(), angle);
        // if robot is not moving, stop the turn motor oscillating
        if (turnAngleError < .5 && Math.abs(state.speedMetersPerSecond) <= 0.03)
            pidOut = 0;

        m_turnMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
    }
    public void setModulePose(Pose2d pose) {
        m_pose = pose;
    }

    public Pose2d getModulePose() {
        return m_pose;
    }

    @Override
    public void periodic() {}
}