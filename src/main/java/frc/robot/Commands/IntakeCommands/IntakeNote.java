package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeNote extends Command{
    
    // Initalizing the Intake Object. 
    private Intake intake; 

    public IntakeNote(Intake intake) { 
        this.intake = intake; 
        addRequirements(intake);    // Makes it so only 1 Intake command can run at any time.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.spinFront(1);
        intake.spinBack(-1);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }  
}
