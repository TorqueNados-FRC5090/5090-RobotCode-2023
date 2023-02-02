package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalOutput;

public class DigitalLED {
    
    private DigitalOutput LED;

    //constructor 
    public DigitalLED(int port){

        LED = new DigitalOutput(port);
    }

    //gets the current value of the LED
    public boolean isOn(){ return LED.get();}
    public boolean isOff(){ return !LED.get();}
    public boolean get(){ return LED.get();}

    //sets the value of the LED
    public void off(){ LED.set(false);}
    public void on(){ LED.set(true);}

    //sets the LED's state to the users will, allows them to change it to whatever
    public void set(boolean set){ LED.set(set);}

    //toggle for the LED
    public void toggle(){
        if(isOn() == true)
            off();
        else 
            on();
    }
}
