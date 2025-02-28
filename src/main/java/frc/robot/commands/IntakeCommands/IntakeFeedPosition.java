package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeFeedPosition extends Command {
    
    private Intake intake; 
    private double setpoint; 

    public IntakeFeedPosition(Intake intake, double setpoint) { 
        this.intake = intake; 
        this.setpoint = setpoint; 
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setPosition(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeStop(); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
