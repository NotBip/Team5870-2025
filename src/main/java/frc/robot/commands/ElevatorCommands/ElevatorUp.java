package frc.robot.commands.ElevatorCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;

public class ElevatorUp extends Command {
   private Elevator elevator; 
    private Supplier<Double> rightTrigger;
    
    public ElevatorUp(Elevator elevator, Supplier<Double> rightTrigger) {
        this.elevator = elevator;
        this.rightTrigger = rightTrigger;
        addRequirements(elevator);
    }
 
    @Override
    public void execute() {
        double speed = rightTrigger.get();
        elevator.elevatorUp(speed);
    }

    @Override
    public void end(boolean interrupted) {
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
