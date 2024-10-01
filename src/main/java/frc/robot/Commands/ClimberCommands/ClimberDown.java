package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class ClimberDown extends Command{

    // Initialzing the Climber Object. 
    private Climber climber; 

    public ClimberDown(Climber climber) { 
        this.climber = climber; 
        addRequirements(climber);   // Makes it so only 1 Climber command can run at any time.  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.climberUp(-.5);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


    
}
