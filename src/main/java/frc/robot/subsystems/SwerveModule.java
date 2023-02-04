// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Pref;
import frc.robot.utils.AngleUtils;
import frc.robot.utils.ShuffleboardContent;

public class SwerveModule extends SubsystemBase {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    public final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkMaxPIDController driveVelController;
    private SparkMaxPIDController turnSMController = null;
    private PIDController turnController = null;
    public final CTRECanCoder turnCANcoder;
    public ModulePosition modulePosition;// enum with test module names;
    SwerveModuleState state;
    public int moduleNumber;
    public String[] modAbrev = { "_FL", "_FR", "_RL", "_RR" };
    String driveLayout;
    String turnLayout;
    String canCoderLayout;
    Pose2d pose;
    double testAngle;
    
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        ModuleConstants.ksVolts,
        ModuleConstants.kvVoltSecondsPerMeter,
        ModuleConstants.kaVoltSecondsSquaredPerMeter);

    private double lastAngle;
    public double angle;

    public double turningEncoderOffset;

    private final int POS_SLOT = 0;
    private final int VEL_SLOT = 1;
    private final int SISLOT = 2;

    private int tuneOn;
    public double actualAngleDegrees;

    private double angleDifference;
    private double angleIncrementPer20ms;
    private double tolDegPerSec = .05;
    private double toleranceDeg = .25;
    public boolean driveMotorConnected;
    public boolean turnMotorConnected;
    public boolean turnCoderConnected;
    private boolean useRRPid =true;
    private double turnDeadband = .5;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel       The channel of the drive motor.
     * @param turningMotorChannel     The channel of the turning motor.
     * @param driveEncoderChannels    The channels of the drive encoder.
     * @param turningCANCoderChannels The channels of the turning encoder.
     * @param driveEncoderReversed    Whether the drive encoder is reversed.
     * @param turningEncoderReversed  Whether the turning encoder is reversed.
     * @param turningEncoderOffset
     */
    public SwerveModule(
        ModulePosition modulePosition,
        int driveMotorCanChannel,
        int turningMotorCanChannel,
        int cancoderCanChannel,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        double turningEncoderOffset) {

        driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(20);
        driveMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        turningMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);
        turningMotor.restoreFactoryDefaults();
        turningMotor.setSmartCurrentLimit(20);
        turningMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);
        turningMotor.setInverted(turningMotorReversed);
        turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        turningMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
        turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // absolute encoder used to establish known wheel position on start position
        turnCANcoder = new CTRECanCoder(cancoderCanChannel);
        turnCANcoder.configFactoryDefault();
        turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());

        this.turningEncoderOffset = turningEncoderOffset;    

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncRPMperMPS);

        driveVelController = driveMotor.getPIDController();

        if (RobotBase.isReal()) {
        driveVelController.setP(.01, VEL_SLOT);
        driveVelController.setD(0, VEL_SLOT);
        driveVelController.setI(0, VEL_SLOT);
        driveVelController.setIZone(1, VEL_SLOT);
        } 
        else {
        driveVelController.setP(1, SISLOT);
        }

        turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

        if (!useRRPid)
        turnSMController = turningMotor.getPIDController();
        else
        turnController = new PIDController(ModuleConstants.kPModuleTurnController, 0, 0);

        if (useRRPid)
        tunePosGains();
        else
        tuneSMPosGains();

        this.modulePosition = modulePosition;
        moduleNumber = modulePosition.ordinal(); // gets module enum index

        if (RobotBase.isSimulation())
        REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));

        checkCAN();

        resetAngleToAbsolute();

        ShuffleboardContent.initDriveShuffleboard(this);
        ShuffleboardContent.initTurnShuffleboard(this);
        ShuffleboardContent.initCANCoderShuffleboard(this);
        ShuffleboardContent.initBooleanShuffleboard(this);
        ShuffleboardContent.initCoderBooleanShuffleboard(this);
    }

    // ----------  Getters  ----------  
    public double getDriveVelocity() { return driveEncoder.getVelocity(); }
    public double getDrivePosition() { return driveEncoder.getPosition(); }
    public double getDriveCurrent() { return driveMotor.getOutputCurrent(); }

    public double getTurnVelocity() { return turningEncoder.getVelocity(); }
    public double getTurnPosition() { return turningEncoder.getPosition(); }
    public double getTurnCurrent() { return turningMotor.getOutputCurrent(); }
    public double getTurnAngle() { return turningEncoder.getPosition(); }

    public ModulePosition getModulePosition() { return modulePosition; }

    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    public double getHeadingDegrees() {
        if (RobotBase.isReal())
        return turningEncoder.getPosition();
        else
        return actualAngleDegrees;
    }

    public SwerveModuleState getState() {
        if (RobotBase.isReal())
        return new SwerveModuleState(driveEncoder.getVelocity(), getHeadingRotation2d());
        else
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(Units.degreesToRadians(angle)));
    }

    // ----------  Setters  ----------  
    public void setModulePose(Pose2d pose) { this.pose = pose; }
    public void setDriveBrakeMode(boolean on) {
        if (on)
        driveMotor.setIdleMode(IdleMode.kBrake);
        else
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    public void setTurnBrakeMode(boolean on) {
        if (on)
        turningMotor.setIdleMode(IdleMode.kBrake);
        else
        turningMotor.setIdleMode(IdleMode.kCoast);
    }

    
    /** Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        state = AngleUtils.optimize(desiredState, getHeadingRotation2d());

        // state = SwerveModuleState.optimize(desiredState, new
        // Rotation2d(actualAngleDegrees));

        // turn motor code
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
            ? lastAngle
            : state.angle.getDegrees();

        lastAngle = angle;

        if (RobotBase.isReal()) {
            // turn axis
            actualAngleDegrees = turningEncoder.getPosition();
            if (useRRPid) {
                positionTurn(angle);
            }

            else {
                positionSMTurn(angle);
            }

            // drive axis
            if (isOpenLoop)
                driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kFreeMetersPerSecond);
            else 
                driveVelController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, VEL_SLOT);
        }

        if (RobotBase.isSimulation()) {
            driveVelController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, SISLOT);

            // no simulation for angle - angle command is returned directly to drive
            // subsystem as actual angle in 2 places - getState() and getHeading
            simTurnPosition(angle);
        }
    }

    // ----------  Others  ----------  

    @Override
    public void periodic() {
        if (Pref.getPref("SwerveTune") == 1 && tuneOn == 0) {
        tuneOn = 1;

        if (useRRPid)
            tunePosGains();
        else
            tuneSMPosGains();
        }

        if (tuneOn == 1)
        tuneOn = (int) Pref.getPref("SwerveTune");

        if (turnCANcoder.getFaulted()) {
        // SmartDashboard.putStringArray("CanCoderFault"
        // + modulePosition.toString(), turnCANcoder.getFaults());
        SmartDashboard.putStringArray("CanCoderStickyFault"
            + modulePosition.toString(), turnCANcoder.getStickyFaults());
        }
    }

    public void tunePosGains() {
        turnController.setP(Pref.getPref("SwerveTurnPoskP"));
        turnController.setI(Pref.getPref("SwerveTurnPoskI"));
        turnController.setD(Pref.getPref("SwerveTurnPoskD"));
    }

    public void tuneSMPosGains() {
        turnSMController.setP(Pref.getPref("SwerveTurnSMPoskP"), POS_SLOT);
        turnSMController.setI(Pref.getPref("SwerveTurnSMPoskI"), POS_SLOT);
        turnSMController.setD(Pref.getPref("SwerveTurnSMPoskD"), POS_SLOT);
        turnSMController.setIZone(Pref.getPref("SwerveTurnSMPoskIz"), POS_SLOT);
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    public static double limitMotorCmd(double motorCmdIn) { return Math.max(Math.min(motorCmdIn, 1.0), -1.0); }
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    public void resetAngleToAbsolute() {
        double angle = turnCANcoder.getAbsolutePosition() - turningEncoderOffset;
        turningEncoder.setPosition(angle);
    }

    
    public void turnMotorMove(double speed) { turningMotor.setVoltage(speed * RobotController.getBatteryVoltage()); }
    public void positionSMTurn(double angle) { turnSMController.setReference(angle, ControlType.kPosition, POS_SLOT); }
    public void positionTurn(double angle) {
        double turnAngleError = Math.abs(angle - turningEncoder.getPosition());

        double pidOut = turnController.calculate(turningEncoder.getPosition(), angle);
        // if robot is not moving, stop the turn motor oscillating
        if (turnAngleError < turnDeadband

            && Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))

        pidOut = 0;

        turningMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());
    }

    private void simTurnPosition(double angle) {
        
        if (angle != actualAngleDegrees && angleIncrementPer20ms == 0) {
        angleDifference = angle - actualAngleDegrees;
        angleIncrementPer20ms = angleDifference / 20; // 10*20ms = .2 sec move time
        }

        if (angleIncrementPer20ms != 0) {
        actualAngleDegrees += angleIncrementPer20ms;

        if ((Math.abs(angle - actualAngleDegrees)) < .1) {
            actualAngleDegrees = angle;
            angleIncrementPer20ms = 0;
        }
        }
    }

    public void driveMotorMove(double speed) { driveMotor.setVoltage(speed * RobotController.getBatteryVoltage()); }
    
    public boolean turnInPosition(double targetAngle) { return Math.abs(targetAngle - getTurnAngle()) < toleranceDeg; }
    public boolean turnIsStopped() { return Math.abs(turningEncoder.getVelocity()) < tolDegPerSec; }

    public boolean checkCAN() {
        driveMotorConnected = driveMotor.getFirmwareVersion() != 0;
        turnMotorConnected = turningMotor.getFirmwareVersion() != 0;
        turnCoderConnected = turnCANcoder.getFirmwareVersion() > 0;

        return driveMotorConnected && turnMotorConnected && turnCoderConnected;
    }
}
