package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;

public class RotateIntakeForward extends Command  {


    private Intake intake;

    public RotateIntakeForward (Intake intake){
        this.intake = intake;
        addRequirements(intake);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.RotateIntakeForward();
      //  System.out.println("RotateIntakeForward");
    }
    


}
