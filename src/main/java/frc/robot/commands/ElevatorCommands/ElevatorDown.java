package frc.robot.commands.ElevatorCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;

public class ElevatorDown extends Command{
    
    private Elevator elevator; 
    private Supplier<Double> leftTrigger;
    
    public ElevatorDown(Elevator elevator, Supplier<Double> leftTrigger) {
        this.elevator = elevator;
        this.leftTrigger = leftTrigger;
        addRequirements(elevator);
    }
 
    @Override
    public void execute() {
        double speed = leftTrigger.get();
        SmartDashboard.putNumber("Elevator down", speed); 
        elevator.elevatorDown(speed);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Elevator down", 0); 
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
