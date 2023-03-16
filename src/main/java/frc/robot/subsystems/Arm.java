package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

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
    private GenericPID telescopePID;
    private GenericPID sliderPID;

    private ArmState currentState = ArmState.ZERO;
    private boolean active = false;
    
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
        rotationPID = new ProfiledPIDController(.1, 0, 0,
            new TrapezoidProfile.Constraints(45, 75));
        rotationPID.setTolerance(1);

        rotationFollower = new CANSparkMax(rotationFollowerId, MotorType.kBrushless);
        rotationFollower.restoreFactoryDefaults();
        rotationFollower.follow(rotation, true);

        telescope = new CANSparkMax(telescopeId, MotorType.kBrushless);
        telescope.restoreFactoryDefaults();
        telescopePID = new GenericPID(telescope, ControlType.kPosition, .025);
        telescopePID.setRatio(TELESCOPE_RATIO);
        
        telescopeFollower = new CANSparkMax(telescopeFollowerId, MotorType.kBrushless);
        telescopeFollower.restoreFactoryDefaults();
        telescopeFollower.follow(telescope, true);

        slider = new CANSparkMax(sliderId, MotorType.kBrushless);
        slider.restoreFactoryDefaults();
        slider.setInverted(true);
        sliderPID = new GenericPID(slider, ControlType.kPosition, .035);
        sliderPID.setRatio(SLIDER_RATIO);
    }

    // Getters
    public CANSparkMax getRotationMotor() { return rotation; }
    public ProfiledPIDController getRotationPid() { return rotationPID; }
    public double getRotationPos() { return rotation.getEncoder().getPosition(); }
    public boolean rotationAtTarget() { return rotationPID.atGoal(); }

    public CANSparkMax getTelescopeMotor() { return telescope; }
    public GenericPID getTelescopePid() { return telescopePID; }
    public double getTelescopePos() { return telescopePID.getPosition(); }

    public CANSparkMax getSliderMotor() { return slider; }
    public GenericPID getSliderPid() { return sliderPID; }
    public double getSliderPos() { return sliderPID.getPosition(); }

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
            case PLACE_HIGH:
                placeHigh();
                break;
        }
    }

    /** @param setpoint The desired angle of the arm */
    private void setRotationSetpoint(double setpoint) { active = true; rotationPID.setGoal(setpoint); }
    /** @param setpoint The desired setpoint for the slider */
    private void setTelescopeSetpoint(double setpoint) { telescopePID.activate(setpoint);}
    /** @param setpoint The desired setpoint for the slider */
    private void setSliderSetpoint(double setpoint) { sliderPID.activate(setpoint); }

    /** Moves the arm to initial position */
    public void zeroPosition(){
        setRotationSetpoint(0);
        setTelescopeSetpoint(0);
        setSliderSetpoint(0);
    }

    /** Move the arm to an intermediate position that 
     *  ensures the arm will not collide with the robot bumper */
    public void intermediatePosition() {
        setRotationSetpoint(20);
        setTelescopeSetpoint(2);
        setSliderSetpoint(4);

    }

    /** Moves the arm to a position ideal for 
     *  taking cargo from the human player */
    public void pickupHuman(){
        setRotationSetpoint(51);
        setTelescopeSetpoint(0);
        setSliderSetpoint(-1);
    }

    /** Moves the arm to a position ideal for 
     *  picking up cargo from the floor */
    public void pickupFloor(){
        setRotationSetpoint(11.3);
        setTelescopeSetpoint(10.4);
        setSliderSetpoint(7);   
    }

    /** Moves the arm to reach middle goal */
    public void dropoffMed(){
        setRotationSetpoint(56);
        setTelescopeSetpoint(6.75);
        setSliderSetpoint(7.25);    
    }

    /** Moves the arm to reach top goal */
    public void dropoffHigh(){
       setRotationSetpoint(62);
       setTelescopeSetpoint(17.75);
       setSliderSetpoint(13.75);
    }

    /** Made to place cones onto pegs from dropoffHigh */
    public void placeHigh() {
        setRotationSetpoint(55);
        setTelescopeSetpoint(17.75);
        setSliderSetpoint(13.75);
    }

    @Override // Called every 20ms
    public void periodic() {
        if(active) {
            double rotationPIDOut = rotationPID.calculate(getRotationPos());
            rotation.setVoltage(rotationPIDOut * RobotController.getBatteryVoltage());
        }
    }
}
