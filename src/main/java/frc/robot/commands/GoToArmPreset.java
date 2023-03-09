package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.subsystems.Arm;

public class GoToArmPreset extends CommandBase{
    
   private Arm arm;
   private ArmState targetState;

   /**constructor
    * @param arm creates dependency on drive train
    * @param target creates a target that can be changed 
    * @param speed the percent speed to drive the robot
    */
    public GoToArmPreset(Arm arm, ArmState targetState){
        
        this.arm = arm;
        this.targetState = targetState;
        
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setTarget(targetState);
    }    

    @Override
    public void execute(){}

    @Override
    public boolean isFinished(){
        return arm.atTarget();
    }

    @Override
    public void end(boolean interrupted){}
}
