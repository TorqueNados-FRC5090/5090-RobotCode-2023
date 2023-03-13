package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForward extends CommandBase{
    
   private Drivetrain drivetrain;

   private double distance; 
   private double target;
   private double speed;

   /** Constructs a new DriveForward command
    * @param drivetrain creates dependency on drive train
    * @param target creates a target that can be changed 
    * @param speed the percent speed to drive the robot
    */
    public DriveForward(Drivetrain drivetrain, double target, double speed){
        this.drivetrain = drivetrain;
        this.target = target;
        this.speed = speed;
        
        addRequirements(drivetrain);
    }

    @Override // Reset Odometry at the start of the command
    public void initialize(){
        drivetrain.resetOdometry(); 
    }    

    @Override
    public void execute(){
        // Drive the robot at the defined speed and direction
        drivetrain.drive(0, Math.abs(speed) * Math.signum(target), 0);
        
        // Update the current distance traveled by the robot
        distance = drivetrain.getPoseMeters().getTranslation().getX();
    }

    @Override // Command ends when the robot has driven far enough
    public boolean isFinished(){
        return Math.abs(distance) >= Math.abs(target);
    }

    @Override // Reset Odometry at the end of the command
    public void end(boolean interrupted){
        drivetrain.resetOdometry();
    }
}
