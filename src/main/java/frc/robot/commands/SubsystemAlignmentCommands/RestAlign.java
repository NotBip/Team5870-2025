package frc.robot.commands.SubsystemAlignmentCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;

public class RestAlign extends Command {

    private Elevator elevator; 
    private Arm arm; 

    public RestAlign(Arm arm, Elevator elevator) { 
        this.elevator = elevator; 
        this.arm = arm; 
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        arm.closeGripper();
    }

    @Override
    public void execute() {
        arm.setPosition(Constants.DeliveryConstants.restPosition);
        if(arm.getArmEncoder() < Constants.DeliveryConstants.restPosition + 5) { 
            elevator.setPoint(Constants.ElevatorConstants.restPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.armStop();
        elevator.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
