package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

// Wraps DigitalInput to represent a laser detector
public class LimitSwitch {
    private DigitalInput limSwitch;

    // Constructor
    public LimitSwitch(int port) {
        limSwitch = new DigitalInput(port);
    }

    // Returns true if the switch is being pressed
    public boolean isPressed() { return !limSwitch.get(); }
}