package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;

public class Level2AutoAlign extends Command {

    private Elevator elevator; 
    private Arm arm; 

    public Level2AutoAlign(Arm arm, Elevator elevator) { 
        this.elevator = elevator; 
        this.arm = arm; 
        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        if(arm.getGripper() == true) { 
            arm.closeGripper();
        }
    }

    @Override
    public void execute() {
        arm.setPosition(Constants.DeliveryConstants.level2Position);
        if(arm.getArmEncoder() < Constants.DeliveryConstants.level2Position + 10.0) { 
            elevator.setPoint(Constants.ElevatorConstants.level2Position);
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
        return false; 
    }
    
}
