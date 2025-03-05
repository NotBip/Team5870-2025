// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToSource;
import frc.robot.commands.ArmCommands.ArmLeft;
import frc.robot.commands.ArmCommands.ArmRight;
import frc.robot.commands.AutoCommands.Level1AutoAlign;
import frc.robot.commands.AutoCommands.Level2AutoAlign;
import frc.robot.commands.AutoCommands.Level3AutoAlign;
import frc.robot.commands.AutoCommands.Level4AutoAlign;
import frc.robot.commands.AutoCommands.RestAutoAlign;
import frc.robot.commands.AutoCommands.SourceIntakeAutoAlign;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.commands.GripperCommands.GripperClose;
import frc.robot.commands.GripperCommands.GripperOpen;
import frc.robot.commands.Swerve.ResetOdometry;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;

public class RobotContainer {

    // Initializing subsystems
    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 
    private Elevator elevator = new Elevator(); 
    private Arm arm = new Arm(); 

    // Initializing Swerve Commands
    private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem);
    private final ResetOdometry resetOdometry = new ResetOdometry(swerveSubsystem);

    // Initatializing Delivery Commands
    private final ArmLeft armLeft = new ArmLeft(arm); 
    private final ArmRight armRight = new ArmRight(arm);  
    private final GripperOpen gripperOpen = new GripperOpen(arm); 
    private final GripperClose gripperClose = new GripperClose(arm); 

    // Initalizing Auto Align Commands
    private final Level1AutoAlign level1AutoAlign = new Level1AutoAlign(arm, elevator); 
    private final Level2AutoAlign level2AutoAlign = new Level2AutoAlign(arm, elevator); 
    private final Level3AutoAlign level3AutoAlign = new Level3AutoAlign(arm, elevator); 
    private final Level4AutoAlign level4AutoAlign = new Level4AutoAlign(arm, elevator); 
    private final SourceIntakeAutoAlign sIntakeAutoAlign = new SourceIntakeAutoAlign(elevator, arm); 
    private final RestAutoAlign restAutoAlign = new RestAutoAlign(arm, elevator); 


    // Initialzing Controllers
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
    
    // Auto April Tag Alignment Systems


    // Game Controllers
    public JoystickButton drBtnA, drBtnB, drBtnX, drBtnY, drBtnLB, drBtnRB, drBtnStrt, drBtnSelect;

    private SendableChooser<Command> sendableChooser = new SendableChooser<Command>(); 

    public RobotContainer() {

        configureNamedCommands();

        sendableChooser.setDefaultOption("NOTHING", null);
        SmartDashboard.putData(sendableChooser);


        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem, 
            () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), 
            () -> !driverJoystick.getRawButton(6), 
            () -> driverController.getRightTriggerAxis() > 0.5 ? true : false));


        // Xbox Driver Controller Buttons
        drBtnA = new JoystickButton(driverJoystick, OIConstants.KXboxButtonA);
        drBtnB = new JoystickButton(driverJoystick, OIConstants.KXboxButtonB);
        drBtnX = new JoystickButton(driverJoystick, OIConstants.KXboxButtonX);
        drBtnY = new JoystickButton(driverJoystick, OIConstants.KXboxButtonY);
        drBtnLB = new JoystickButton(driverJoystick, OIConstants.KXboxLeftBumper);
        drBtnRB = new JoystickButton(driverJoystick, OIConstants.KXboxRightBumper);
        drBtnSelect = new JoystickButton(driverJoystick, OIConstants.KXboxSelectButton);
        drBtnStrt = new JoystickButton(driverJoystick, OIConstants.KXboxStartButton);

        configureBindings(); 
    }

    private void configureBindings() {
        drBtnStrt.onTrue(zeroGyro);
        drBtnSelect.onTrue(resetOdometry); 

        driverController.a().whileTrue(new AutoAlignToSource(swerveSubsystem, false, false));

        operatorController.axisGreaterThan(2, .1).whileTrue(new ElevatorDown(elevator, () -> operatorController.getRawAxis(2)));
        operatorController.axisGreaterThan(3, .1).whileTrue(new ElevatorUp(elevator, () -> operatorController.getRawAxis(3)));

        operatorController.rightBumper().whileTrue(armLeft); 
        operatorController.leftBumper().whileTrue(armRight); 

        operatorController.povUp().whileTrue(level4AutoAlign); 
        operatorController.povLeft().whileTrue(level3AutoAlign); 
        operatorController.povDown().whileTrue(level2AutoAlign); 
        operatorController.povRight().whileTrue(level1AutoAlign); 

        operatorController.y().whileTrue(sIntakeAutoAlign); 
        operatorController.x().onTrue(gripperOpen); 
        operatorController.b().onTrue(gripperClose);
        operatorController.a().whileTrue(restAutoAlign); 

    }

    public void updateArmPID(double P, double I, double D) { 
        arm.setArmPID(P, I, D);
    }

    public void updateElevatorPID(double P, double I, double D) { 
        elevator.setElevatorPID(P, I, D);
    }

    public void configureNamedCommands() { 
        // NamedCommands.registerCommand("ZeroGyro", zeroGyro);
    }



    public Command getAutonomousCommand() {
        return sendableChooser.getSelected();

    }

    public Command selfTestCommand() {
        return null;
        // return new SwerveJoystickCmd(swerveSubsystem, () -> 0.0, () -> 0.0, () -> 2.0, () -> false, () -> false);
    }
}
