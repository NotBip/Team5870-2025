// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ClimberCommands.ClimberDown;
import frc.robot.Commands.ClimberCommands.ClimberStop;
import frc.robot.Commands.ClimberCommands.ClimberUp;
import frc.robot.Commands.IntakeCommands.IntakeNote;
import frc.robot.Commands.IntakeCommands.IntakeStop;
import frc.robot.Commands.IntakeCommands.ShootNote;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;

public class RobotContainer {

    // Initializing the Subsystems. 
    private Climber climber = new Climber(); 
    private Intake intake = new Intake(); 

    // Initializing the Climber Commands. 
    private ClimberUp climberUp = new ClimberUp(climber); 
    private ClimberStop climberStop = new ClimberStop(climber);
    private ClimberDown climberDown = new ClimberDown(climber);

    // Initializing the Intake Commands
    private IntakeNote intakeNote = new IntakeNote(intake); 
    private IntakeStop intakeStop = new IntakeStop(intake); 
    private ShootNote shootNote = new ShootNote(intake); 

    // Initalizing the Controllers. 
    private CommandXboxController opController = new CommandXboxController(1); 

    public RobotContainer() {
        // setting the default commands for the subsystems.
        climber.setDefaultCommand(climberStop);
        intake.setDefaultCommand(intakeStop);

        configureBindings(); 
    }

    /**
     * This method should be used when binding controller to commands. This method should be called at the END OF ROBOT CONTAINER
     */
    private void configureBindings() {
        opController.y().whileTrue(climberUp); // Assigning the Controller button "Y" to moving the climber up at a constant speed of .5
        opController.a().whileTrue(climberDown); // Assigning the Controller button "A" to moving the climber down at a constant speed of .5

        opController.rightBumper().whileTrue(intakeNote); // Assigning the Left Bumper to intake note
        opController.leftBumper().whileTrue(shootNote); // Assigning the Right Bumper to shoot note. 
    }

    
    public Command getAutonomousCommand() {
        return null;
    }

}
