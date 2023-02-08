package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

/** Wraps DigitalInput to represent a laser detector */
public class LaserDetector {
    // Create a digital input for the switch
    private DigitalInput laser;

    /** 
     * Constructs LaserDetector object 
     * 
     * @param port The number of the DIO input
     */
    public LaserDetector(int port) {
        laser = new DigitalInput(port);
    }

    /** @return true if something is blocking the laser */
    public boolean isBlocked() { return !laser.get(); }

    /** @return true if nothing is blocking the laser */
    public boolean isOpen() { return laser.get(); }
}
