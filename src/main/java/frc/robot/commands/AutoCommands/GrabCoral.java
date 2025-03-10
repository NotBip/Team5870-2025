package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Arm.Arm; 
import frc.robot.Constants; 

public class GrabCoral extends Command {

    private Elevator elevator;
    private Arm arm;
    private boolean isDone;  
    
    public GrabCoral(Elevator elevator, Arm arm){
        this.elevator = elevator;
        this.arm = arm; 
        addRequirements(elevator, arm);
    }

    @Override
    public void initialize() {
        arm.openGripper();
        isDone = false; 
    }

    @Override
    public void execute() {
        if(arm.getGripper() == false) { 
            arm.openGripper();
        }
        
       elevator.setPoint(Constants.ElevatorConstants.grabPosition);
       if(elevator.getElevatorEncoder() > Constants.ElevatorConstants.grabPosition - 5) { 
        arm.setPosition(Constants.DeliveryConstants.grabPosition);
       }

       if(arm.getArmEncoder() > 41) { 
            isDone = true; 
       }
    }


    @Override
    public void end(boolean interrupted) {
        arm.closeGripper();
        elevator.stopElevator();
        arm.armStop();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}


