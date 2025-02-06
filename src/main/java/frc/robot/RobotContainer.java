// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.commands.Swerve.ResetOdom;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;

public class RobotContainer {

    // Initializing subsystems
    public SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 

    // Initializing Swerve Commands
    private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem);
    private final ResetOdom resetOdom = new ResetOdom(swerveSubsystem); 

    // Initialzing Controllers
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort); 


    // Game Controllers
    public JoystickButton drBtnA, drBtnB, drBtnX, drBtnY, drBtnLB, drBtnRB, drBtnStrt, drBtnSelect;

    private SendableChooser<Command> sendableChooser = new SendableChooser<Command>(); 

    public RobotContainer() {




        configureNamedCommands();

        sendableChooser.setDefaultOption("NOTHING", null);
        sendableChooser.addOption("drive Straight", AutoBuilder.buildAuto("Straight"));
        sendableChooser.addOption("drive straight 180", AutoBuilder.buildAuto("180"));
        sendableChooser.addOption("diagonalstuff", AutoBuilder.buildAuto("complexauto"));
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
        drBtnA.onTrue(resetOdom);
    }

    

    public void configureNamedCommands() { 
        NamedCommands.registerCommand("ZeroGyro", zeroGyro);
        // NamedCommands.registerCommand("", resetOdom);
    }

    public Command getAutonomousCommand() {


        try { 
            PathPlannerPath newPath = PathPlannerPath.fromPathFile("path 1");
            return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                sendableChooser.getSelected()
            );
            
        } catch (Exception e)  {
            System.out.println("Failed to get Path.");
            return null; 
        }

    }

    public Command selfTestCommand() { 
        return new SwerveJoystickCmd(swerveSubsystem, () -> 0.0, () -> 0.0, () -> 2.0, () -> false, () -> false);
    }
}
