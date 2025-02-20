// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Elevator.changeToLevel0;
import frc.robot.Commands.Elevator.moveMotorDown;
import frc.robot.Commands.Elevator.moveMotorUp;
import frc.robot.Commands.Elevator.stopMotor;
import frc.robot.Commands.Elevator.changeToLevel0;
import frc.robot.Commands.Elevator.changeToLevel1;
import frc.robot.Commands.Elevator.changeToLevel2;
import frc.robot.Commands.Elevator.changeToLevel3;
import frc.robot.Commands.Elevator.changeToLevel4;


import frc.robot.Subsystems.Elevator;

public class RobotContainer {

  private CommandXboxController xboxController = new CommandXboxController(0);

  //Subsystems.
  private Elevator elevator = new Elevator(); 
  
  //Elevator commands.
  private moveMotorDown moveMotorDown = new moveMotorDown(elevator);
  private moveMotorUp moveMotorUp = new moveMotorUp(elevator);
  private stopMotor stopMotor = new stopMotor(elevator);
  private changeToLevel0 toLevel0 = new changeToLevel0(elevator);
  private changeToLevel1 toLevel1 = new changeToLevel1(elevator);
  private changeToLevel2 toLevel2 = new changeToLevel2(elevator);
  private changeToLevel3 toLevel3 = new changeToLevel3(elevator);
  private changeToLevel4 toLevel4 = new changeToLevel4(elevator);
  


  public RobotContainer() {

    elevator.setDefaultCommand(stopMotor);
    configureBindings();
  }

  private void configureBindings() {

    xboxController.a().whileTrue(moveMotorUp);
    xboxController.b().whileTrue(moveMotorDown);

    xboxController.x().onTrue(toLevel0);
    xboxController.povUp().onTrue(toLevel1);
    xboxController.povRight().onTrue(toLevel2);
    xboxController.povDown().onTrue(toLevel3);
    xboxController.povLeft().onTrue(toLevel4);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
