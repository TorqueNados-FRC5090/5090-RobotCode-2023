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
    private CANSparkMax telescope;
    private CANSparkMax telescopeFollower;
    private CANSparkMax slider;

    // Declare GenericPID objects to control the motors
    private GenericPID rotationPID;
    private GenericPID telescopePID;
    private GenericPID sliderPID;
    
    /**
     * Constructs Arm subsystem
     * 
     * @param rotationId ID of the rotation motor
     * @param telescopeId ID of the telescope motor
     * @param telescopeFollowerId ID of the telescope follower motor
     * @param sliderId ID of the slider motor
     */
     public Arm(int rotationId, int telescopeId, int telescopeFollowerId, int sliderId){

        rotation = new CANSparkMax(rotationId, MotorType.kBrushless);
        rotation.restoreFactoryDefaults();
        rotationPID = new GenericPID(rotation,ControlType.kPosition , .5);
        rotationPID.setRatio(ROTATION_RATIO);
        rotationPID.setInputRange(0,90);        

        telescope = new CANSparkMax(telescopeId, MotorType.kBrushless);
        telescope.restoreFactoryDefaults();
        telescopePID = new GenericPID(telescope, ControlType.kPosition, .5);
        telescopePID.setInputRange(0,12); 
        
        telescopeFollower = new CANSparkMax(telescopeFollowerId, MotorType.kBrushless);
        telescopeFollower.restoreFactoryDefaults();
        telescopeFollower.follow(telescope, true);

        slider = new CANSparkMax(sliderId, MotorType.kBrushless);
        slider.restoreFactoryDefaults();
        slider.setInverted(true);
        sliderPID = new GenericPID(slider, ControlType.kPosition, .25);
        sliderPID.setInputRange(0,12); 

    }

    // Getters
    public CANSparkMax getRotationMotor(){ return rotation; }
    public CANSparkMax getTelescopeMotor() { return telescope; }
    public CANSparkMax getSliderMotor() { return slider; }
    public GenericPID getRotationPid(){ return rotationPID; }
    public GenericPID getTelescopePid(){ return telescopePID; }
    public GenericPID getSliderPid() { return sliderPID; }

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


    /** Sets arm to initial position */
    public void zeroPosition(){
        rotationGoTo(0);
        telescopeGoTo(0);
        sliderGoTo(0);
    }

    /** Sets arm to reach top goal */
    public void topPosition(){
        rotationGoTo(45);
        telescopeGoTo(36);
        sliderGoTo(24);
    }

    /** Sets arm to reach middle goal */
    public void middlePosition(){
        rotationGoTo(25);
        telescopeGoTo(18);
        sliderGoTo(12);    
    }

    /** Sets arm to reach bottom goal */
    public void bottomPosition(){
        rotationGoTo(-5);
        telescopeGoTo(0);
        sliderGoTo(12);   
    }

    /** Moves the arm to the most balanced postition */
    public void balance(){
        rotationGoTo(-5);
        telescopeGoTo(0);
        sliderGoTo(12);   
    }
}
