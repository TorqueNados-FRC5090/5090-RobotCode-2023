package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

// Wraps DigitalInput to represent a laser detector
public class LaserDetector {
    private DigitalInput laser;

    // Constructor
    public LaserDetector(int port) {
        laser = new DigitalInput(port);
    }

    // Returns true if something is blocking the laser
    public boolean isBlocked() { return !laser.get(); }

    // Returns true if nothing is blocking the laser
    public boolean isOpen() { return laser.get(); }
}
