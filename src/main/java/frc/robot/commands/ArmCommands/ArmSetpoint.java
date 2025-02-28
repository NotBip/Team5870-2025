package frc.robot.commands.ArmCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;

public class ArmSetpoint extends Command{
    
    private Arm arm; 
    private double setpoint; 

    public ArmSetpoint(Arm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint; 
        addRequirements(arm);
    }
 
    @Override
    public void execute() {
        arm.setPosition(setpoint); 
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
