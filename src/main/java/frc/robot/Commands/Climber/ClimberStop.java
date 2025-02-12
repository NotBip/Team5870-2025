package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Climber;

public class ClimberStop extends Command{

    private Climber climber; 
    

    public ClimberStop(Climber climber) { 
        this.climber = climber; 
        addRequirements(climber);
    }



    @Override
    public void execute() {
        climber.stop();
    }
    
}
