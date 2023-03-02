package frc.robot.subsystems;

// WPI imports
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Motor related imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder;

// Math imports
import frc.robot.utils.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;

// Import constants
import static frc.robot.Constants.SwerveConstants.ModuleConstants.*;
import static frc.robot.Constants.SwerveConstants.MAX_TRANSLATION_SPEED;

/** This class represents a single swerve module */
public class SwerveModule extends SubsystemBase {
    private final int VEL_SLOT = 1;
    private int moduleNumber;
    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;
    private SwerveModuleState state;
    private SparkMaxPIDController driveController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;
    private PIDController turnController;
    private CANCoder angleEncoder;
    private double angleOffset;
    private double m_lastAngle;
    private Pose2d pose;

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
        int driveMotorID,
        int turnMotorID,
        int absoluteEncoderID,
        boolean driveMotorInverted,
        boolean turningMotorInverted,
        double turningEncoderOffset) {

        // Initialize internal variables with values passed through params
        this.moduleNumber = moduleNumber;
        angleOffset = turningEncoderOffset;

        // Construct and configure the driving motor
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(40);
        driveMotor.getPIDController().setFF(0.0);
        driveMotor.getPIDController().setP(0.2);
        driveMotor.getPIDController().setI(0.0);
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        driveMotor.enableVoltageCompensation(12.6);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Initialize the driving motor's encoder
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DRIVE_REVS_TO_M);
        driveEncoder.setVelocityConversionFactor(DRIVE_RPM_TO_MPS);

        // Initialize the driving motor's PID controller
        driveController = driveMotor.getPIDController();

        // Construct and configure the turning motor
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setSmartCurrentLimit(20);
        turnMotor.getPIDController().setFF(0.0);
        turnMotor.getPIDController().setP(0.2);
        turnMotor.getPIDController().setI(0.0);
        turnMotor.setInverted(turningMotorInverted);
        turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        turnMotor.enableVoltageCompensation(12.6);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Initialize the driving motor's encoder
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(TURNING_REVS_TO_DEG);
        turnEncoder.setVelocityConversionFactor(TURNING_REVS_TO_DEG / 60);

        // Construct a PID controller to help turn the module to a direction
        turnController = new PIDController(.007, .00175, .0000625);

        // Construct and initialize the absolute encoder
        angleEncoder = new CANCoder(absoluteEncoderID);
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());

        // Point the module forward
        resetAngleToAbsolute();
    }

    /** Useful for iterating over modules like an array
     *  @return the number of this module */
    public int getModuleNumber() { return moduleNumber; }
    /** @return the direction this module is facing in degrees */
    public double getHeadingDegrees() { return turnEncoder.getPosition(); }
    /** @return the direction this module is facing as a {@link Rotation2d} object */
    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    /** @return How far this module has driven total in meters */
    public double getDriveMeters() { return driveEncoder.getPosition(); }
    /** @return The current speed of this module in m/sec */
    public double getDriveMetersPerSecond() { return driveEncoder.getVelocity(); }
    /** @return The current {@link SwerveModuleState state} of this module */
    public SwerveModuleState getState() { return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d()); }
    /** @return The {@link SwerveModulePosition position} of this module 
     *  expressed as the total distance driven and current heading */
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d()); }
    /** @return The current {@link Pose2d pose} of this module */
    public Pose2d getModulePose() { return pose; }
    /** Sets this module's {@link Pose2d pose} */
    public void setModulePose(Pose2d pose) { this.pose = pose; }
    

    /** Set the turning motor's encoder to absolute zero */
    public void resetAngleToAbsolute() {
        double angle = angleEncoder.getAbsolutePosition() - angleOffset;
        turnEncoder.setPosition(angle);
    }

      /**Resets this modules drive encoder*/
      public void resetDriveEncoder(){
        driveEncoder.setPosition(0);
    }

    /**Resets this modules turn encoder */
    public void resetTurnEncoder(){
        turnEncoder.setPosition(0);
    }

    /**Resets both turn and drive encoders for this module */
    public void resetEncoders(){
        resetTurnEncoder();
        resetDriveEncoder();
    }

    /** Set the entire module to a desired {@link SwerveModuleState state}, controlling
     *  both the direction and speed at the same time
     * 
     *  @param desiredState The {@link SwerveModuleState state} to set the module to
     *  @param isOpenLoop True to control the driving motor via %power.
     *                    False to control the driving motor via velocity-based PID.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        state = RevUtils.optimize(desiredState, getHeadingRotation2d());

        if (isOpenLoop) {
            // Calculate the %power for the driving motor
            double percentOutput = state.speedMetersPerSecond / MAX_TRANSLATION_SPEED;
            // Send instruction to the motor
            driveMotor.set(percentOutput);
        } 
        else {
            // Set the driving motor's PID controller to the desired speed
            int DRIVE_PID_SLOT = VEL_SLOT;
            driveController.setReference(
                state.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                DRIVE_PID_SLOT
            );
        }

        // Get the angle to turn the module to
        double angle =
            (Math.abs(state.speedMetersPerSecond) <= (MAX_TRANSLATION_SPEED * 0.01))
                ? m_lastAngle
                : state.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    
        // Point turning motor at the target angle
        turnTo(angle);
    }

    /** Turn the module to point in some direction
     * 
     *  @param angle the target angle in degrees
     */
    public void turnTo(double angle) {
        double turnAngleError = Math.abs(angle - turnEncoder.getPosition());

        double pidOut = turnController.calculate(turnEncoder.getPosition(), angle);
        // if robot is not moving, stop the turn motor oscillating
        if (turnAngleError < .5 && Math.abs(state.speedMetersPerSecond) <= 0.03)
            pidOut = 0;

        turnMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());
    }
}