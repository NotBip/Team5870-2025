// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSim;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;

public class RobotContainer {

    // Initializing subsystems
    public SwerveSim swerveSubsystem = new SwerveSim(); 

    // Initializing Swerve Commands
    private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem);

    // Initialzing Controllers
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort); 

    // Climber Sim Values
    double z = -.2; 
    double y = 0;

    // Algae Rotation
    double algaeIntakeRot = 0; 

    // GAME PIECES

    double xRot = 0; 

    // ALGAE 1
    double x1 = 1.25;
    double y1 = 6.5;

    // ALGAE 2
    double x2 = 1.25; 
    double y2 = 4.5; 

    // ALGAE 3
    double x3 = 1.25; 
    double y3 = 2.5; 
    


    // Game Controllers
    public JoystickButton drBtnA, drBtnB, drBtnX, drBtnY, drBtnLB, drBtnRB, drBtnStrt, drBtnSelect;

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> driverJoystick.getRawAxis(3), 
            () -> driverJoystick.getRawAxis(2), 
            () -> driverJoystick.getRawAxis(2) == 1 ? true : false, 
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));


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



    public void simMovement() { 

        // CLIMBER
        if(driverController.getRightTriggerAxis() > .5) { 
            System.out.println("RIGHT");
           z += .01;
        }

        if(driverController.getLeftTriggerAxis() > .5) { 
            System.out.println("L:EFT");
            z -= .01;
        }

        if (z > 0.6) z = .6; 
        if (z < -0.2) z = -0.2;

        if(driverController.getLeftBumperButton()) { 
            System.out.println("RIGHT");
            y += .01; 
        }

        if(driverController.getRightBumperButton()) { 
            System.out.println("L:EFT");
            y -= .01; 
        }
        if (y > .370) y = .370;
        if (y < 0) y = 0; 
        
        // ALGAE INTAKE
        if (driverController.getPOV() == 90) { 
            algaeIntakeRot -= 0.02;    
        } 
        if (driverController.getPOV() == 270) { 
            algaeIntakeRot += 0.02;    
        }

        if (algaeIntakeRot > .15) algaeIntakeRot = .15; 
        if (algaeIntakeRot < -.15) algaeIntakeRot = -.15; 



        // ALGAE
        Logger.recordOutput("Algae Sim", new Pose3d[] { 
            new Pose3d(x1, y1, .2, new Rotation3d(xRot, xRot, xRot)),
            new Pose3d(x2, y2, .2, new Rotation3d(xRot, xRot, xRot)),
            new Pose3d(x3, y3, .2, new Rotation3d(xRot, xRot, xRot))
        });  
        
        double gpDist1 = dist(swerveSubsystem.getPose(), new Pose2d(x1, y1, new Rotation2d()));
        double gpDist2 = dist(swerveSubsystem.getPose(), new Pose2d(x2, y2, new Rotation2d()));
        double gpDist3 = dist(swerveSubsystem.getPose(), new Pose2d(x3, y3, new Rotation2d()));

        xRot = swerveSubsystem.getPose().getRotation().getRadians();

        if(gpDist1 < .25 && algaeIntakeRot < -.1) { 
            x1 = swerveSubsystem.getPose().getX() - .3; 
            y1 = swerveSubsystem.getPose().getY(); 
        } else if (gpDist2 < .25 && algaeIntakeRot < -.1) { 
            x2 = swerveSubsystem.getPose().getX() - .3; 
            y2 = swerveSubsystem.getPose().getY(); 
        } else if (gpDist3 < .25 && algaeIntakeRot < -.1) { 
            x3 = swerveSubsystem.getPose().getX() - .3; 
            y3 = swerveSubsystem.getPose().getY(); 
        }
        // Logger.recordOutput("Algae Sim", new Pose3d(1.25, 4.5, .2, new Rotation3d(0, 0, 0)));        
        // Logger.recordOutput("Algae Sim", new Pose3d(1.25, 2.5, .2, new Rotation3d(0, 0, 0)));        


        Logger.recordOutput("Final Component Poses", new Pose3d[] { 
        // Climber
        new Pose3d(
            0 , 0, z, new Rotation3d(0,0,0)
        ), 
        new Pose3d(
            0, y, z, new Rotation3d(0, 0, 0)
        ), 
        // Algae Intake
        new Pose3d(
            0, 0, 0, new Rotation3d(0, algaeIntakeRot, 0)
        )
        });
    }

    public void configureNamedCommands() { 
        // NamedCommands.registerCommand("ZeroGyro", zeroGyro);
    }

    public double dist(Pose2d robotPose, Pose2d objPose) { 
        double a = Math.pow((objPose.getX() - robotPose.getX()), 2); 
        double b = Math.pow((objPose.getY() - robotPose.getY()), 2);
        
        // return Math.sqrt(Math.abs(a + b)); 
        return Math.abs(Math.hypot(a, b));
    }

    public Command getAutonomousCommand() {

        try { 
            PathPlannerPath path = PathPlannerPath.fromPathFile("Straight");
            return AutoBuilder.followPath(path); 
        } catch (Exception e)  {
            System.out.println("Failed to get Path.");
            return null; 
        }

    }
}
