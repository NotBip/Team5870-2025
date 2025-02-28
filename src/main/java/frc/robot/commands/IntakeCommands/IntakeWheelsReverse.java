package frc.robot.commands.IntakeCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeWheelsReverse extends Command {
    
        private Intake intake; 
        private Supplier<Double> leftTrigger; 
    
        public IntakeWheelsReverse(Intake intake, Supplier<Double> leftTrigger) { 
            this.intake = intake; 
            this.leftTrigger = leftTrigger; 
            addRequirements(intake);
        }
    
        @Override
        public void initialize() {
    
        }
    
        @Override
        public void execute() {
            double speed = leftTrigger.get(); 
            intake.spinWheelsReverse(speed);
        }
    
        @Override
        public void end(boolean interrupted) {
            intake.intakeWheelsStop();
        }
    
        @Override
        public boolean isFinished() {
            return false; 
        }
        
        

}
