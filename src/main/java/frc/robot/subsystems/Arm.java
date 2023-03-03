package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.wrappers.GenericPID;

import static frc.robot.Constants.ArmConstants.*;

/** This class is used to control the robot's arm */
public class Arm {

    // Declare motors used by the arm
    private CANSparkMax rotation;
    private CANSparkMax rotationFollower;
    private CANSparkMax telescope;
    private CANSparkMax telescopeFollower;
    private CANSparkMax slider;

    // Declare GenericPID objects to control the motors
    private GenericPID rotationPID;
    private GenericPID telescopePID;
    private GenericPID sliderPID;

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
        rotationPID = new GenericPID(rotation,ControlType.kPosition , .025);
        rotationPID.setRatio(ROTATION_RATIO);
        //rotationPID.setInputRange(0,80);        

        rotationFollower = new CANSparkMax(rotationFollowerId, MotorType.kBrushless);
        rotationFollower.restoreFactoryDefaults();
        rotationFollower.follow(rotation, true);

        telescope = new CANSparkMax(telescopeId, MotorType.kBrushless);
        telescope.restoreFactoryDefaults();
        telescopePID = new GenericPID(telescope, ControlType.kPosition, .025);
        telescopePID.setRatio(TELESCOPE_RATIO);
        //telescopePID.setInputRange(0,17); 
        
        telescopeFollower = new CANSparkMax(telescopeFollowerId, MotorType.kBrushless);
        telescopeFollower.restoreFactoryDefaults();
        telescopeFollower.follow(telescope, true);

        slider = new CANSparkMax(sliderId, MotorType.kBrushless);
        slider.restoreFactoryDefaults();
        slider.setInverted(true);
        sliderPID = new GenericPID(slider, ControlType.kPosition, .035);
        sliderPID.setRatio(SLIDER_RATIO);
        //sliderPID.setInputRange(0,14); 

    }

    // Getters
    public CANSparkMax getRotationMotor(){ return rotation; }
    public CANSparkMax getTelescopeMotor() { return telescope; }
    public CANSparkMax getSliderMotor() { return slider; }
    public GenericPID getRotationPid(){ return rotationPID; }
    public GenericPID getTelescopePid(){ return telescopePID; }
    public GenericPID getSliderPid() { return sliderPID; }
    public ArmState getCurrentState() { return currentState; }

    /** Rotates the arm to a specific position 
     * @param target The target position in degrees */
    public void rotationGoTo(double target){
        rotationPID.activate(target);
    }  
    /** Extends arm to a specific length 
     *  @param target The target length in inches */
    public void telescopeGoTo(double target){    
        telescopePID.activate(target);
    }
    /** Slides arm to a specific position on the chassis
     *  @param target The target position in inches */
    public void sliderGoTo(double target){
        sliderPID.activate(target);
    }

    /**  */
    public void goTo(ArmState preset) {
        currentState = changeState(preset);

        switch(currentState) {
            case ZERO:
                zeroPosition();
                break;
            case BALANCE:
                balancePosition();
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

    /** Moves the arm to the most balanced postition */
    public void balancePosition(){
          
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

    /** Handles situations where the arm will collide with the robot's bumpers 
     * 
     * @param targetState The state the arm wants to go to
     * @return An {@link ArmState intermediate state} if the arm will collide.
     *         The target {@link ArmState state} otherwise */
    private ArmState changeState(ArmState targetState){
        return willConflict(targetState)
            ? ArmState.INTERMEDIATE : targetState;
    }

    /** Compares the current state to the target to see if there will be a
     *  collision between the arm and the bumper of the robot
     * 
     *  @param targetState The state the arm wants to go to
     *  @return Whether a collision will occur */
    private boolean willConflict(ArmState targetState) {
        if((currentState == ArmState.ZERO || currentState == ArmState.BALANCE)
        && (targetState == ArmState.PICKUP_FLOOR))
            return true;
        else if((targetState == ArmState.ZERO || targetState == ArmState.BALANCE)
        && (currentState == ArmState.PICKUP_FLOOR))    
            return true;
        else
            return false;
    }
}
