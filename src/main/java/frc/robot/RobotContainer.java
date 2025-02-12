// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Elevator.moveMotorDown;
import frc.robot.Commands.Elevator.moveMotorUp;
import frc.robot.Commands.Elevator.stopMotor;


import frc.robot.Subsystems.Elevator;

public class RobotContainer {

  private CommandXboxController xboxController = new CommandXboxController(0);

  //Subsystems.
  private Elevator elevator = new Elevator(); 
  
  //Elevator commands.
  private moveMotorDown moveMotorDown = new moveMotorDown(elevator);
  private moveMotorUp moveMotorUp = new moveMotorUp(elevator);
  private stopMotor stopMotor = new stopMotor(elevator);


  public RobotContainer() {

    elevator.setDefaultCommand(stopMotor);
    configureBindings();
  }

  private void configureBindings() {

    xboxController.a().whileTrue(moveMotorUp);
    xboxController.b().whileTrue(moveMotorDown);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
