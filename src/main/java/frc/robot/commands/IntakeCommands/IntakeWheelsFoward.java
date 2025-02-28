package frc.robot.commands.IntakeCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;

public class IntakeWheelsFoward extends Command {
    
        private Intake intake; 
        private Supplier<Double> rightTrigger; 
    
        public IntakeWheelsFoward(Intake intake, Supplier<Double> rightTrigger) { 
            this.intake = intake; 
            this.rightTrigger = rightTrigger; 
            addRequirements(intake);
        }
    
        @Override
        public void initialize() {
    
        }
    
        @Override
        public void execute() {
            double speed = rightTrigger.get(); 
            intake.spinWheelsFoward(speed);
            System.out.println(speed);
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
