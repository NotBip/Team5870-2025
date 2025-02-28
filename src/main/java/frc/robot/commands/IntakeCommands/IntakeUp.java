package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeUp extends Command {

    private Intake intake; 

    public IntakeUp(Intake intake) { 
        this.intake = intake; 
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.rotateIntakeBack();
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
