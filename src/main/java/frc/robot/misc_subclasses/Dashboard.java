package frc.robot.misc_subclasses;

// Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.wrappers.GenericPID;

/** This class is used to handle {@link SmartDashboard} outputs.
 *  The purpose of this class is to centralize code related to the dashboard. */
public class Dashboard {
    /** Constructs a Dashboard object */
    public Dashboard() {}

    /** Prints the current position of a target relative to limelight 
     *  @param limelight The limelight object to get data from */
    public void printLimelightData(Limelight limelight) {
        SmartDashboard.putNumber("Distance from Target", limelight.getDistance());
        SmartDashboard.putNumber("Rotational Angle to Target", limelight.getRotationAngle());
    }

    /** Prints the position and state of the arm
     *  @param arm The robot's arm */
    public void printArmData(Arm arm) {
        PIDtoDashboard(arm.getRotationPid(), "Arm Rotator");
        PIDtoDashboard(arm.getTelescopePid(), "Arm Telescope");
        PIDtoDashboard(arm.getSliderPid(), "Arm Slider");

        SmartDashboard.putString("Arm State", arm.getCurrentState().toString());
    }

    /** Prints the heading of the robot and whether it is in field centric mode or not 
     *  @param drivetrain The robot's drivetrain */
    public void printBasicDrivetrainData(Drivetrain drivetrain) {
        SmartDashboard.putBoolean("Field Centric", drivetrain.isFieldCentric());
        SmartDashboard.putNumber("Robot Heading", drivetrain.getHeadingDegrees());
        SmartDashboard.putString("Robot Pose", drivetrain.getPoseMeters().toString());
	    SmartDashboard.putBoolean("Heading Controller At Setpoint", drivetrain.headingPIDAtTarget());
    }

    /**
    * Prints relevant data from a PID controller. If a value is manually 
    * input into the dashboard, the pid setpoint will update accordingly
    *
    * @param pid The {@link GenericPID} to get data from
    * @param name A name for the GenericPID on the dashboard
    */
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
        SmartDashboard.putNumber(name + " Position", pid.getRatioPos());
        SmartDashboard.putString(name + " Domain", "[" + pid.getMin() + ", " + pid.getMax() + "]");
    }
}
