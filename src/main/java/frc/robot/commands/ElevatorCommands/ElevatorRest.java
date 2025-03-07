package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Arm.Arm; 
import frc.robot.Constants; 
public class ElevatorRest extends Command {

    private Elevator elevator;
    private Arm arm; 
    private double setpoint;
    
    public ElevatorRest(Elevator elevator, double setpoint, Arm arm){
        this.elevator = elevator;
        this.setpoint = setpoint;
        this.arm = arm; 
        addRequirements(elevator);
    }

    @Override
    public void end(boolean interrupted) {
       elevator.stopElevator();
       arm.armStop();
       arm.closeGripper();
    }

    @Override
    public void execute() {
        
       elevator.setPoint(setpoint);
       if(elevator.getElevatorEncoder() > 93) { 
        arm.setPosition(Constants.DeliveryConstants.grabPosition);
       }
    }

    @Override
    public void initialize() {
        arm.openGripper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
