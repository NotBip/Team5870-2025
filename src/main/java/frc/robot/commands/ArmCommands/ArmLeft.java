package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;

public class ArmLeft extends Command{
    
    private Arm arm; 
    
    public ArmLeft(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }
 
    @Override
    public void execute() {
        arm.rotateArm(); 
    }

    @Override
    public void end(boolean interrupted) {
        arm.armStop();
    }   

    @Override
    public void initialize() {
       
    }

    @Override
    public boolean isFinished() {
       return false;
    }


}
