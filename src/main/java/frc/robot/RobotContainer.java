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
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToSource;
import frc.robot.commands.ArmCommands.ArmLeft;
import frc.robot.commands.ArmCommands.ArmRight;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.commands.GripperCommands.GripperClose;
import frc.robot.commands.GripperCommands.GripperOpen;
import frc.robot.commands.SubsystemAlignmentCommands.Level1Align;
import frc.robot.commands.SubsystemAlignmentCommands.Level2Align;
import frc.robot.commands.SubsystemAlignmentCommands.Level3Align;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;
import frc.robot.commands.SubsystemAlignmentCommands.RestAlign;
import frc.robot.commands.SubsystemAlignmentCommands.SourceIntakeAlign;
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
    private final Level1Align level1Align = new Level1Align(arm, elevator); 
    private final Level2Align level2Align = new Level2Align(arm, elevator); 
    private final Level3Align level3Align = new Level3Align(arm, elevator); 
    private final Level4Align level4Align = new Level4Align(arm, elevator); 
    private final SourceIntakeAlign sIntakeAlign = new SourceIntakeAlign(elevator, arm); 
    private final RestAlign restAlign = new RestAlign(arm, elevator); 


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
        driverController.b().whileTrue(new AutoAlignToReef(swerveSubsystem, 6, true)); 
        driverController.x().whileTrue(new AutoAlignToReef(swerveSubsystem, 6, false)); 

        operatorController.axisGreaterThan(2, .1).whileTrue(new ElevatorDown(elevator, () -> operatorController.getRawAxis(2)));
        operatorController.axisGreaterThan(3, .1).whileTrue(new ElevatorUp(elevator, () -> operatorController.getRawAxis(3)));

        operatorController.rightBumper().whileTrue(armLeft); 
        operatorController.leftBumper().whileTrue(armRight); 

        operatorController.povUp().whileTrue(level4Align); 
        operatorController.povLeft().whileTrue(level3Align); 
        operatorController.povDown().whileTrue(level2Align); 
        operatorController.povRight().whileTrue(level1Align); 

        operatorController.y().whileTrue(sIntakeAlign); 
        operatorController.x().onTrue(gripperOpen); 
        operatorController.b().onTrue(gripperClose);
        operatorController.a().whileTrue(restAlign); 

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
