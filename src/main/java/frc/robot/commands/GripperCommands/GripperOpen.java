package frc.robot.commands.GripperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.Arm;

public class GripperOpen extends Command {

    private Arm arm; 
    
    public GripperOpen(Arm arm) { 
        this.arm = arm; 
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        arm.openGripper();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
