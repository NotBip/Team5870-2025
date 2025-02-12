package frc.robot.Commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Pneumatics;

//extends pneumatics pistons

public class RotatingArmRelease extends Command  {


    private Pneumatics pneumatics;

    public RotatingArmRelease (Pneumatics pneumatics){
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pneumatics.RotatingArmGrab();
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
    


}
