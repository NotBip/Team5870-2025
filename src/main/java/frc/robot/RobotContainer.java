// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Subsystems.Intake;
import frc.robot.Commands.Intake.RotateIntakeBackward;
import frc.robot.Commands.Intake.RotateIntakeForward;
import frc.robot.Commands.Intake.RotateWheels;
import frc.robot.Commands.Intake.RotateWheelsReverse;
import frc.robot.Commands.Intake.Stop;

import frc.robot.Subsystems.Delivery;
import frc.robot.Commands.Delivery.RotatingArm;

import frc.robot.Subsystems.Pneumatics;
import frc.robot.Commands.Pneumatics.RotatingArmGrab;
import frc.robot.Commands.Pneumatics.RotatingArmRelease;

public class RobotContainer {

  private CommandXboxController xboxController = new CommandXboxController(0);
  
  private Intake intake = new Intake();
  private Delivery delivery = new Delivery();
  private Pneumatics pneumatics = new Pneumatics();

  private RotateIntakeBackward RotateIntakeBackward = new RotateIntakeBackward(intake);
  private RotateIntakeForward RotateIntakeForward = new RotateIntakeForward(intake);
  private RotateWheels RotateWheels = new RotateWheels(intake);
  private RotateWheelsReverse RotateWheelsReverse = new RotateWheelsReverse(intake);
  private Stop Stop = new Stop(intake);

  private RotatingArm RotatingArm = new RotatingArm(delivery);

  private RotatingArmRelease RotatingArmRelease = new RotatingArmRelease(pneumatics);
  private RotatingArmGrab RotatingArmGrab = new RotatingArmGrab(pneumatics);

  public RobotContainer() {

    intake.setDefaultCommand(Stop);

    configureBindings();
  }

  private void configureBindings() {

   //xboxController.a().onTrue(RotateIntakeForward);
    xboxController.a().whileTrue(RotateIntakeForward);

   //xboxController.b().onTrue(RotateWheels);
    xboxController.b().whileTrue(RotateWheels);

    //xboxController.x().onTrue(RotateIntakeBackward);
    xboxController.x().whileTrue(RotateIntakeBackward);


    //xboxController.y().onTrue(RotateWheelsReverse);
    xboxController.y().whileTrue(RotateWheelsReverse);



    // xboxController.a().onTrue(RotatingArmRelease);
    // xboxController.a().whileTrue(RotatingArmRelease);

    // xboxController.b().onTrue(RotatingArmGrab);
    // xboxController.b().whileTrue(RotatingArmGrab);

    // xboxController.x().onTrue(RotatingArm);
    // xboxController.x().whileTrue(RotatingArm);
  }

  public Command getAutonomousCommand() {
  
    return Commands.print("No autonomous command configured");
  }
}
