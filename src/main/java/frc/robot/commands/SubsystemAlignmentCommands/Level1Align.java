package frc.robot.commands.SubsystemAlignmentCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;

public class Level1Align extends Command {
    
    private Arm arm; 
    private Elevator elevator;

    public Level1Align(Arm arm, Elevator elevator) { 
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
        arm.setPosition(Constants.DeliveryConstants.level1Position);
        if(arm.getArmEncoder() < Constants.DeliveryConstants.level1Position + 10.0) { 
            elevator.setPoint(Constants.ElevatorConstants.level1Position);
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
