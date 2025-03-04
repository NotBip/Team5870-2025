package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Arm.Arm; 
import frc.robot.Constants; 

public class SourceIntakeAutoAlign extends Command {

    private Elevator elevator;
    private Arm arm; 
    
    public SourceIntakeAutoAlign(Elevator elevator, Arm arm){
        this.elevator = elevator;
        this.arm = arm; 
        addRequirements(elevator, arm);
    }

    @Override
    public void initialize() {
        arm.openGripper();
    }

    @Override
    public void execute() {
        
       elevator.setPoint(Constants.ElevatorConstants.restPosition);
       if(elevator.getElevatorEncoder() > 100) { 
        arm.setPosition(Constants.DeliveryConstants.restPosition);
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
        return false;
    }
}
