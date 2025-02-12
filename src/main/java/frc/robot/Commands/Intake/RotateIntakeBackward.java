package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;

public class RotateIntakeBackward extends Command  {


    private Intake intake;

    public RotateIntakeBackward (Intake intake){
        this.intake = intake;
        addRequirements(intake);
       // System.out.println("RotateIntakeBackwards");

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.RotateIntakeBackward();
    }
    


}
