package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
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
        rotationPID.setSetpoint(0);
        telescopePID.setSetpoint(0);
        sliderPID.setSetpoint(0);
    }

    /** Move the arm to an intermediate position that 
     *  ensures the arm will not collide with the robot bumper */
    public void intermediatePosition() {
        rotationPID.setSetpoint(25);
        telescopePID.setSetpoint(1);
        sliderPID.setSetpoint(4.5);

    }

    /** Moves the arm to a position ideal for 
     *  taking cargo from the human player */
    public void pickupHuman(){
        rotationPID.setSetpoint(75);
        telescopePID.setSetpoint(0);
        sliderPID.setSetpoint(-1);
    }

    /** Moves the arm to a position ideal for 
     *  picking up cargo from the floor */
    public void pickupFloor(){
        rotationPID.setSetpoint(10);
        telescopePID.setSetpoint(11);
        sliderPID.setSetpoint(9.5);   
    }

    /** Moves the arm to reach middle goal */
    public void dropoffMed(){
        rotationPID.setSetpoint(80);
        telescopePID.setSetpoint(7);
        sliderPID.setSetpoint(5.5);    
    }

    /** Moves the arm to reach top goal */
    public void dropoffHigh(){
       rotationPID.setSetpoint(96);
       telescopePID.setSetpoint(16);
       sliderPID.setSetpoint(14);
    }

    /** Moves the arm to a position defined by parameters */
    public void testPosition(double rotation, double tele, double slider){
        rotationPID.setSetpoint(rotation);
        telescopePID.setSetpoint(tele);
        sliderPID.setSetpoint(slider);
    }

    @Override // Called every 20ms
    public void periodic() {
        double rotationPIDOut = rotationPID.calculate(getRotationPos());
        double telescopePIDOut = telescopePID.calculate(getTelescopePos());
        double sliderPIDOut = sliderPID.calculate(getSliderPos());

        rotation.setVoltage(rotationPIDOut * RobotController.getBatteryVoltage());
        telescope.setVoltage(telescopePIDOut * RobotController.getBatteryVoltage());
        slider.setVoltage(sliderPIDOut * RobotController.getBatteryVoltage());
    }
}
