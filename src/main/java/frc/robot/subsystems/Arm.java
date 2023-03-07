package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.ArmConstants.*;

/** This class is used to control the robot's arm */
public class Arm {

    // Declare motors used by the arm
    private CANSparkMax rotation;
    private CANSparkMax rotationFollower;
    private CANSparkMax telescope;
    private CANSparkMax telescopeFollower;
    private CANSparkMax slider;

    // Declare PID controllers to control the motors
    private PIDController rotationPID;
    private PIDController telescopePID;
    private PIDController sliderPID;

    private ArmState currentState = ArmState.ZERO;
    
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
        rotationPID = new PIDController(.025, 0, 0);
        rotationPID.setTolerance(1);

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
    public PIDController getRotationPid() { return rotationPID; }
    public double getRotationPos() { return rotation.getEncoder().getPosition(); }
    public boolean rotationAtTarget() { return rotationPID.atSetpoint(); }

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

    /** Rotates the arm to a specific position 
     * @param target The target position in degrees */
    public void rotationGoTo(double target){
        double pidOut = rotationPID.calculate(getRotationPos(), target);
        rotation.set(pidOut);
    }  
    /** Extends arm to a specific length 
     *  @param target The target length in inches */
    public void telescopeGoTo(double target){    
        double pidOut = telescopePID.calculate(getTelescopePos(), target);
        telescope.set(pidOut);
    }
    /** Slides arm to a specific position on the chassis
     *  @param target The target position in inches */
    public void sliderGoTo(double target){
        double pidOut = sliderPID.calculate(getSliderPos(), target);
        slider.set(pidOut);
    }

    /** Send the arm to a target state and update the state
     *  @param preset The state to go to
     */
    public void goTo(ArmState preset) {
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

    /** Moves the arm to initial position */
    public void zeroPosition(){
        rotationGoTo(0);
        telescopeGoTo(0);
        sliderGoTo(0);
    }

    /** Move the arm to an intermediate position that 
     *  ensures the arm will not collide with the robot bumper */
    public void intermediatePosition() {
        rotationGoTo(25);
        telescopeGoTo(1);
        sliderGoTo(4.5);

    }

    /** Moves the arm to a position ideal for 
     *  taking cargo from the human player */
    public void pickupHuman(){
        rotationGoTo(75);
        telescopeGoTo(0);
        sliderGoTo(-1);
    }

    /** Moves the arm to a position ideal for 
     *  picking up cargo from the floor */
    public void pickupFloor(){
        rotationGoTo(10);
        telescopeGoTo(11);
        sliderGoTo(9.5);   
    }

    /** Moves the arm to reach middle goal */
    public void dropoffMed(){
        rotationGoTo(80);
        telescopeGoTo(7);
        sliderGoTo(5.5);    
    }

    /** Moves the arm to reach top goal */
    public void dropoffHigh(){
       rotationGoTo(96);
       telescopeGoTo(16);
       sliderGoTo(14);
    }

    /** Moves the arm to a position defined by parameters */
    public void testPosition(double rotation, double tele, double slider){
        rotationGoTo(rotation);
        telescopeGoTo(tele);
        sliderGoTo(slider);
    }
}
