package frc.robot.commands.ElevatorCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;

public class StopElevator extends Command{
    private Elevator elevator; 
    
    public StopElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
 
    @Override
    public void execute() {
        elevator.stopElevator();
    }


    @Override
    public void initialize() {
       
    }

    @Override
    public boolean isFinished() {
       return false;
    }


}
