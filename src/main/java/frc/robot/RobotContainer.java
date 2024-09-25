// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.commands.Climber.ClimberManualPosition;
import frc.commands.Climber.ClimberStop;
import frc.commands.Intake.IntakeSpinForward;
import frc.commands.Intake.IntakeStop;
import frc.commands.Swerve.SwerveJoystickCmd;
import frc.commands.Swerve.ZeroGyro;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 
    private Climber climber = new Climber(); 
    private Intake intake = new Intake(); 
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort); 
  
    private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem); 

    private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake); 
    private final IntakeStop intakeStop = new IntakeStop(intake);
    
    private final ClimberStop climberStop = new ClimberStop(climber); 


    // Game Controllers
    public JoystickButton drBtnA, drBtnB, drBtnX, drBtnY, drBtnLB, drBtnRB, drBtnStrt, drBtnSelect;



    public RobotContainer() {
        configureNamedCommands();
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem, 
            () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), 
            () -> !driverJoystick.getRawButton(6), 
            () -> driverController.getRightTriggerAxis() > 0.5 ? true : false));

        intake.setDefaultCommand(intakeStop);
        climber.setDefaultCommand(climberStop);

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

        
    }

    public void configureNamedCommands() { 
        NamedCommands.registerCommand("ShootIntake", intakeSpinForward.withTimeout(.75));  
        NamedCommands.registerCommand("ZeroGyro", zeroGyro);
        NamedCommands.registerCommand("ArmDown", new ClimberManualPosition(climber, 0).withTimeout(2));
        NamedCommands.registerCommand("ArmUp", new ClimberManualPosition(climber, 50).withTimeout(2));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("amp+mob"))),
            new InstantCommand(() -> swerveSubsystem.zeroHeading()), 
            new PathPlannerAuto("amp+mob"));
    }
}
