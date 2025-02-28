// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.ArmCommands.ArmLeft;
import frc.robot.commands.ArmCommands.ArmLevel1;
import frc.robot.commands.ArmCommands.ArmLevel2;
import frc.robot.commands.ArmCommands.ArmLevel3;
import frc.robot.commands.ArmCommands.ArmLevel4;
import frc.robot.commands.ArmCommands.ArmRest;
import frc.robot.commands.ArmCommands.ArmRight;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.ElevatorCommands.ElevatorLevel1;
import frc.robot.commands.ElevatorCommands.ElevatorLevel2;
import frc.robot.commands.ElevatorCommands.ElevatorLevel3;
import frc.robot.commands.ElevatorCommands.ElevatorLevel4;
import frc.robot.commands.ElevatorCommands.ElevatorRest;
import frc.robot.commands.ElevatorCommands.ElevatorUp;
import frc.robot.commands.GripperCommands.GripperClose;
import frc.robot.commands.GripperCommands.GripperOpen;
import frc.robot.commands.IntakeCommands.IntakeDown;
import frc.robot.commands.IntakeCommands.IntakeFeedPosition;
import frc.robot.commands.IntakeCommands.IntakeRest;
import frc.robot.commands.IntakeCommands.IntakeUp;
import frc.robot.commands.IntakeCommands.IntakeWheelsFoward;
import frc.robot.commands.IntakeCommands.IntakeWheelsReverse;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;

public class RobotContainer {

    // Initializing subsystems
    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 
    private Elevator elevator = new Elevator(); 
    private Intake intake = new Intake(); 
    private Arm arm = new Arm(); 

    // Initializing Swerve Commands
    private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem);

    // Initatializing Arm Commands
    private final ArmLeft armLeft = new ArmLeft(arm); 
    private final ArmLevel1 armLevel1 = new ArmLevel1(arm, Constants.DeliveryConstants.level1Position); 
    private final ArmLevel2 armLevel2 = new ArmLevel2(arm, Constants.DeliveryConstants.level2Position);
    private final ArmLevel3 armLevel3 = new ArmLevel3(arm, Constants.DeliveryConstants.level3Position); 
    private final ArmLevel4 armLevel4 = new ArmLevel4(arm, Constants.DeliveryConstants.level4Position);
    private final ArmRest armRest = new ArmRest(arm, Constants.DeliveryConstants.restPosition); 
    private final ArmRight armRight = new ArmRight(arm); 

    // Initializing Elevator Commands
    private final ElevatorLevel1 elevatorLevel1 = new ElevatorLevel1(elevator, Constants.ElevatorConstants.level1Position); 
    private final ElevatorLevel2 elevatorLevel2 = new ElevatorLevel2(elevator, Constants.ElevatorConstants.level2Position);
    private final ElevatorLevel3 elevatorLevel3 = new ElevatorLevel3(elevator, Constants.ElevatorConstants.level3Position);
    private final ElevatorLevel4 elevatorLevel4 = new ElevatorLevel4(elevator, Constants.ElevatorConstants.level4Position);
    private final ElevatorRest elevatorRest = new ElevatorRest(elevator, Constants.ElevatorConstants.restPosition);
    
    // Initializing  Gripper Commmands
    // private final GripperClose gripperClose = new GripperClose(arm); 
    // private final GripperOpen gripperOpen = new GripperOpen(arm); 

    // Initializing Intake Commands
    private final IntakeFeedPosition intakeFeedPosition = new IntakeFeedPosition(intake, Constants.IntakeConstants.feedPosition); 
    private final IntakeRest intakeRest = new IntakeRest(intake, Constants.IntakeConstants.groundPosition); 
    private final IntakeUp intakeUp = new IntakeUp(intake); 
    private final IntakeDown intakeDown = new IntakeDown(intake); 
    private final GripperOpen gripperOpen = new GripperOpen(arm); 
    private final GripperClose gripperClose = new GripperClose(arm); 


    // Initialzing Controllers
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort); 


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
        driverController.axisGreaterThan(2, .1).whileTrue(new ElevatorDown(elevator, () -> driverController.getRawAxis(2)));
        driverController.axisGreaterThan(3, .1).whileTrue(new ElevatorUp(elevator, () -> driverController.getRawAxis(3)));

        driverController.leftBumper().whileTrue(armLeft); 
        driverController.rightBumper().whileTrue(armRight); 

        driverController.povLeft().whileTrue(intakeDown); 
        driverController.povRight().whileTrue(intakeUp); 

        
        driverController.povUp().onTrue(gripperOpen); 
        driverController.povDown().onTrue(gripperClose);

        driverController.y().whileTrue(elevatorLevel4); 
        // operatorController.axisGreaterThan(2, .1).whileTrue(new IntakeWheelsFoward(intake, () -> operatorController.getRawAxis(2)));
        // operatorController.axisGreaterThan(3, .1).whileTrue(new IntakeWheelsReverse(intake, () -> operatorController.getRawAxis(3)));


    }

    public void updateArmPID(double P, double I, double D) { 
        arm.setArmPID(P, I, D);
    }

    public void updateElevatorPID(double P, double I, double D) { 
        elevator.setElevatorPID(P, I, D);
    }

    public void updateIntakePID(double P, double I, double D) { 
        intake.setAnglePID(P, I, D);
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
