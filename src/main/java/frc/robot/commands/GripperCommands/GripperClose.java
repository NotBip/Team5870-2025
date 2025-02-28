package frc.robot.commands.GripperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.Arm;

public class GripperClose extends Command {

    private Arm arm; 
    
    public GripperClose(Arm arm) { 
        this.arm = arm; 
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        arm.closeGripper();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
