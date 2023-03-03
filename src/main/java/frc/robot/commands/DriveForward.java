package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForward extends CommandBase{
    
   private Drivetrain drivetrain;

   private double distance; 
   private double target;
   private double speed;


   /**constructor
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

    @Override
    public void initialize(){
        drivetrain.resetOdometry();
    }    

    @Override
    public void execute(){
        drivetrain.drive(0, Math.abs(speed) * Math.signum(target), 0, true);
        distance = drivetrain.getPoseMeters().getTranslation().getX();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(distance) >= Math.abs(target);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.resetOdometry();
    }
}
