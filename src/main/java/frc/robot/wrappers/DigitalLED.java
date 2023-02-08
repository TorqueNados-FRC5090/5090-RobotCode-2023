package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalOutput;

/** 
 * This class wraps over a {@link DigitalOutput} that represents an LED in the real world.
 */ 
public class DigitalLED {
    
    private DigitalOutput LED;

    /**
     * Constructs a DigitalLED object
     * 
     * @param port The DIO port that we use
     */
    public DigitalLED(int port){

        LED = new DigitalOutput(port);
    }

    // Gets the current value of the LED
    public boolean isOn(){ return LED.get();}
    public boolean isOff(){ return !LED.get();}
    public boolean get(){ return LED.get();}

    // Sets the value of the LED
    public void off(){ LED.set(false);}
    public void on(){ LED.set(true);}

    /**@param set If set to true, the LED's will be turned on*/
    public void set(boolean set){ LED.set(set);}

    /** This function sets the light to the opposite of its current state */
    public void toggle(){
        set(!get());
    }
}
