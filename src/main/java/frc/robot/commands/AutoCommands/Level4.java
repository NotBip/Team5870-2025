package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Intake.Intake;

public class Level4 extends Command {

    private Arm arm; 
    private Elevator elevator; 

    public Level4(Arm arm, Elevator elevator) { 
        this.arm = arm; 
        this.elevator = elevator; 
    }
    

    @Override
    public void initialize() {
        
    }

    
    @Override
    public void execute() {
    }


    @Override
    public void end(boolean interrupted) {
        
    }



    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
