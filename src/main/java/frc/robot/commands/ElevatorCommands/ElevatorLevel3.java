package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;

public class ElevatorLevel3 extends Command {

    private Elevator elevator;
    private double setpoint;
    
    public ElevatorLevel3(Elevator elevator, double setpoint){
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void end(boolean interrupted) {
       elevator.stopElevator();
    }

    @Override
    public void execute() {
       elevator.setPoint(setpoint);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
