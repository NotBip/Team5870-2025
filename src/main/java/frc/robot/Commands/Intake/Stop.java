package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;

public class Stop extends Command  {


    private Intake intake;

    public Stop (Intake intake){
        this.intake = intake;
        addRequirements(intake);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.Stop();
    }
    


}
