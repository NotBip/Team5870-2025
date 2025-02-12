package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class moveMotorDown extends Command{

    private Elevator elevator;

    public moveMotorDown(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.moveMotorDown();
    }

    
}
