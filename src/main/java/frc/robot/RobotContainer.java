// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.Climber;
import frc.robot.Commands.Climber.ClimberDown;
import frc.robot.Commands.Climber.ClimberStop;
import frc.robot.Commands.Climber.MoveUp;


public class RobotContainer {

  private CommandXboxController xboxController = new CommandXboxController(0); 

  // Subsystems. 
  private Climber climber = new Climber(); 
  
  // Climber commands
  private MoveUp moveUp = new MoveUp(climber); 
  private ClimberStop climberStop = new ClimberStop(climber); 
  private ClimberDown climberDown = new ClimberDown(climber);



  public RobotContainer() {

    climber.setDefaultCommand(climberStop);

    configureBindings();
  }

  private void configureBindings() {
    xboxController.y().whileTrue(moveUp);   
    xboxController.a().whileTrue(climberDown); 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
