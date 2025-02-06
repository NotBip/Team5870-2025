package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Climber;

public class MoveUp extends Command {

    private Climber climber; 

    public MoveUp(Climber climber) { 
        this.climber = climber; 
        addRequirements(climber);
    }

    @Override
    public void initialize() {
      
    }

    @Override
    public void execute() {
        climber.moveUp(); 
    }


    
}
