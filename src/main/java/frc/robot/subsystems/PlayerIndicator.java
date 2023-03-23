package frc.robot.subsystems;

import frc.robot.wrappers.DigitalLED;

public class PlayerIndicator {
    
    private DigitalLED LED;

    public PlayerIndicator(int digitalLEDPort){

        LED = new DigitalLED(digitalLEDPort);

    }

    /**  */
    public DigitalLED getLED(){ return LED;}
    public boolean isCube() { return LED.isOn(); }
    public boolean isCone() { return LED.isOff(); }
    public String getStateAsString() { return isCube() ? "CUBE" : "CONE"; }


    public void indicatorToggle(){
        LED.toggle();
    }

    public void setCube(){
        LED.set(true);
    }

    public void setCone(){
        LED.set(false);
    }



}
