package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

/** This class is used to control the robot's arm */
public class Arm extends SubsystemBase {

    // Declare motors used by the arm
    private CANSparkMax rotation;
    private CANSparkMax rotationFollower;
    private CANSparkMax telescope;
    private CANSparkMax telescopeFollower;
    private CANSparkMax slider;

    // Declare PID controllers to control the motors
    private ProfiledPIDController rotationPID;
    private ArmFeedforward rotationFF;
    private PIDController telescopePID;
    private PIDController sliderPID;

    private ArmState currentState = ArmState.ZERO;
    /** The last recorded velocity of the rotation motor */
    private double prevRotVel = 0;
    private double prevTime = Timer.getFPGATimestamp();
    
    /**
     * Constructs Arm subsystem
     * 
     * @param rotationId ID of the rotation motor
     * @param rotationFollowerId ID of the rotation follower motor
     * @param telescopeId ID of the telescope motor
     * @param telescopeFollowerId ID of the telescope follower motor
     * @param sliderId ID of the slider motor
     */
     public Arm(int rotationId, int rotationFollowerId, int telescopeId, int telescopeFollowerId, int sliderId){

        rotation = new CANSparkMax(rotationId, MotorType.kBrushless);
        rotation.restoreFactoryDefaults();
        rotation.getEncoder().setPositionConversionFactor(ROTATION_RATIO);
        rotationPID = new ProfiledPIDController(.05, 0, 0, 
            new TrapezoidProfile.Constraints(30, 40));
        rotationPID.setTolerance(1);
        rotationFF = new ArmFeedforward(.01, 0, .00, 0);

        rotationFollower = new CANSparkMax(rotationFollowerId, MotorType.kBrushless);
        rotationFollower.restoreFactoryDefaults();
        rotationFollower.follow(rotation, true);

        telescope = new CANSparkMax(telescopeId, MotorType.kBrushless);
        telescope.restoreFactoryDefaults();
        telescope.getEncoder().setPositionConversionFactor(TELESCOPE_RATIO);
        telescopePID = new PIDController(.025, 0, 0);
        telescopePID.setTolerance(.15);
        
        telescopeFollower = new CANSparkMax(telescopeFollowerId, MotorType.kBrushless);
        telescopeFollower.restoreFactoryDefaults();
        telescopeFollower.follow(telescope, true);

        slider = new CANSparkMax(sliderId, MotorType.kBrushless);
        slider.restoreFactoryDefaults();
        slider.setInverted(true);
        slider.getEncoder().setPositionConversionFactor(SLIDER_RATIO);
        sliderPID = new PIDController(.035, 0, 0);
        sliderPID.setTolerance(.15);

    }

    // Getters
    public CANSparkMax getRotationMotor() { return rotation; }
    public ProfiledPIDController getRotationPid() { return rotationPID; }
    public double getRotationPos() { return rotation.getEncoder().getPosition(); }
    public boolean rotationAtTarget() { return rotationPID.atGoal(); }

    public CANSparkMax getTelescopeMotor() { return telescope; }
    public PIDController getTelescopePid() { return telescopePID; }
    public double getTelescopePos() { return telescope.getEncoder().getPosition(); }
    public boolean telescopeAtTarget() { return telescopePID.atSetpoint(); }

    public CANSparkMax getSliderMotor() { return slider; }
    public PIDController getSliderPid() { return sliderPID; }
    public double getSliderPos() { return slider.getEncoder().getPosition(); }
    public boolean sliderAtTarget() { return sliderPID.atSetpoint(); }

    public boolean atTarget() {
        return rotationAtTarget()
            && telescopeAtTarget()
            && sliderAtTarget();
    }
    public ArmState getCurrentState() { return currentState; }

    /** Move the arm to one of its presets */
    public void setTarget(ArmState preset) {
        currentState = preset;

        switch(preset) {
            case ZERO:
                zeroPosition();
                break;
            case INTERMEDIATE:
                intermediatePosition();
                break;
            case PICKUP_FLOOR:
                pickupFloor();
                break;
            case PICKUP_HUMAN:
                pickupHuman();
                break;
            case DROPOFF_MED:
                dropoffMed();
                break;
            case DROPOFF_HIGH:
                dropoffHigh();
                break;
        }
    }

    /** @param setpoint The desired angle of the arm */
    private void setRotationSetpoint(double setpoint) { rotationPID.setGoal(setpoint); }
    /** @param setpoint The desired setpoint for the telescope */
    private void setTelescopeSetpoint(double setpoint) {}
    /** @param setpoint The desired setpoint for the slider */
    private void setSliderSetpoint(double setpoint) {}

    /** Moves the arm to initial position */
    public void zeroPosition(){
        setRotationSetpoint(0);
        setTelescopeSetpoint(0);
        setSliderSetpoint(0);
    }

    /** Move the arm to an intermediate position that 
     *  ensures the arm will not collide with the robot bumper */
    public void intermediatePosition() {
        setRotationSetpoint(25);
        setTelescopeSetpoint(1);
        setSliderSetpoint(4.5);

    }

    /** Moves the arm to a position ideal for 
     *  taking cargo from the human player */
    public void pickupHuman(){
        setRotationSetpoint(75);
        setTelescopeSetpoint(0);
        setSliderSetpoint(-1);
    }

    /** Moves the arm to a position ideal for 
     *  picking up cargo from the floor */
    public void pickupFloor(){
        setRotationSetpoint(10);
        setTelescopeSetpoint(11);
        setSliderSetpoint(9.5);   
    }

    /** Moves the arm to reach middle goal */
    public void dropoffMed(){
        setRotationSetpoint(80);
        setTelescopeSetpoint(7);
        setSliderSetpoint(5.5);    
    }

    /** Moves the arm to reach top goal */
    public void dropoffHigh(){
       setRotationSetpoint(96);
       setTelescopeSetpoint(16);
       setSliderSetpoint(14);
    }

    @Override // Called every 20ms
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();

        // Get each motor's PID output
        double rotationPIDOut = rotationPID.calculate(getRotationPos());
        double telescopePIDOut = telescopePID.calculate(getTelescopePos());
        double sliderPIDOut = sliderPID.calculate(getSliderPos());

        // Get the arm's velocity
        double rotationVel = rotationPID.getSetpoint().velocity;
        // Calculate the arm's acceleration
        double rotationAccel = (rotationVel - prevRotVel) / (currentTime - prevTime);
        // Calculate rotation feed forward output
        double rotationFFOut = rotationFF.calculate(getRotationPos() - 60, rotationVel/*, rotationAccel*/);

        rotation.setVoltage((rotationPIDOut + rotationFFOut) * RobotController.getBatteryVoltage());
        telescope.setVoltage(telescopePIDOut * RobotController.getBatteryVoltage());
        slider.setVoltage(sliderPIDOut * RobotController.getBatteryVoltage());

        // Used for tuning, TODO should probably be moved or deleted later
        SmartDashboard.putNumber("rotationVel", rotationVel);
        SmartDashboard.putNumber("rotationAccel", rotationAccel);
        SmartDashboard.putNumber("rotationPIDOut", rotationPIDOut);
        SmartDashboard.putNumber("rotationFFOut", rotationFFOut);
        SmartDashboard.putNumber("(rotationPIDOut + rotationFFOut)", (rotationPIDOut + rotationFFOut));

        // Update internal values
        prevRotVel = rotationVel;
        prevTime = currentTime;
    }
}
