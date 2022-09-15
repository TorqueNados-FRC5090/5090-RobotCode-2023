package frc.robot.misc_subclasses;

// Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.GenericPID;

// This object is used to handle SmartDashboard outputs
public class Dashboard {
    // Constructor method
    public Dashboard() {}

    // Prints the current position of a target relative to limelight
    public void printLimelightData(Limelight limelight) {
        SmartDashboard.putNumber("Distance from Target", limelight.getDistance());
        SmartDashboard.putNumber("Rotational Angle to Target", limelight.getRotationAngle());
    }

    // Prints relevant data from a PID controller. If a value is manually 
    // input into the dashboard, the pid setpoint will update accordingly
    public void PIDtoDashboard(GenericPID pid, String name) {
        // Get the setpoint from the dashboard
        double setpointD = SmartDashboard.getNumber(name + " Setpoint", pid.getSetpoint());

        // If the setpoint on the dashboard does not match the pid 
        // setpoint, then the setpoint on the dashboard is used
        if( setpointD != pid.getSetpoint() )
            pid.setSetpoint(setpointD);
            
        // Print useful info to the dashboard
        SmartDashboard.putNumber(name + " Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber(name + " RPM", pid.getRPM());
        SmartDashboard.putNumber(name + " Position", pid.getPosition());
        SmartDashboard.putString(name + " Domain", "[" + pid.getMin() + ", " + pid.getMax() + "]");
    }
}
