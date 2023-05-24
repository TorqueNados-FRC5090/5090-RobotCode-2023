package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.wrappers.LaserDetector;

public class Claw {

    private LaserDetector laserDetector;
    private DoubleSolenoid dubs; 
    private Boolean isOpen = false;

    public Claw(int laserPort){

        dubs = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

        laserDetector = new LaserDetector(laserPort);
    }

    public DoubleSolenoid getDoubleSolenoid(){return dubs;}
    public Boolean isOpen(){return isOpen;}

    public void toggleClaw(){
        if(isOpen == false){
            dubs.set(Value.kForward);
            isOpen = true;
        }   
        else if(isOpen == true){
            dubs.set(Value.kReverse);
            isOpen = false;
        }
    }
    public void open(){
        dubs.set(Value.kForward);
        isOpen = true;
    }
    public void close(){
        dubs.set(Value.kReverse);
        isOpen = false;
    }

    public void autoGrab(){
        if(laserDetector.isBlocked() == true){ 
            close();
        }
    }
}
