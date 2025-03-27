package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;

public class AutoLevel4Align extends Command {

    private Elevator elevator; 
    private Arm arm; 
    private boolean isDone; 

    public AutoLevel4Align(Arm arm, Elevator elevator) { 
        this.elevator = elevator; 
        this.arm = arm; 
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        isDone = false; 
        if(arm.getGripper() == true) { 
            arm.closeGripper();
        }
    }

    @Override
    public void execute() {
        arm.setPosition(Constants.DeliveryConstants.level4Position);
        elevator.setPoint(Constants.ElevatorConstants.level4Position);
        if(arm.getArmEncoder() > 18) { 
            isDone = true; 
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.openGripper();
        arm.armStop();
        elevator.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return isDone; 
    }
    
}
