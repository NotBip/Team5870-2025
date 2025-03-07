package frc.robot.commands.SubsystemAlignmentCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Arm.Arm; 
import frc.robot.Constants; 

public class SourceIntakeAlign extends Command {

    private Elevator elevator;
    private Arm arm; 
    
    public SourceIntakeAlign(Elevator elevator, Arm arm){
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
        
       elevator.setPoint(Constants.ElevatorConstants.grabPosition);
       if(elevator.getElevatorEncoder() > 90) { 
        arm.setPosition(Constants.DeliveryConstants.grabPosition);
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
