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


    // Sim Values 
    double innerZ = .96; 
    double outerZ = .84; 

    double intakeRot = -.04; 
    double armRot = 0.0; 
    


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

        if (driverController.getRightBumperButton()) { 
            intakeRot += 0.06; 
        } else if (driverController.getLeftBumperButton()) { 
            intakeRot -= .06; 
        }

        if (driverController.getBButton()) { 
            armRot += .06; 
        } else if (driverController.getXButton()) { 
            armRot -= .06;
        }

        if(driverController.getYButton()) { 
            if (outerZ - innerZ < .1) { 
                innerZ += .01;
                outerZ += .02;
            } else { 
                innerZ += .01; 
            }
        }
        if(driverController.getAButton()) { 
            if (outerZ - innerZ > .5) { 
                innerZ -= .01;
                outerZ -= .02;
            } else { 
                innerZ -= .01; 
            }

            if (innerZ <= 0) { 
                outerZ -= .02;
            }

        }

        if (innerZ > .96) innerZ = .96; 
        if (innerZ < 0) innerZ = 0; 
        if (outerZ > .84) outerZ = .84; 
        if (outerZ < 0) outerZ = 0; 
        if (intakeRot < -2.4) intakeRot = -2.4; 
        if (intakeRot > -0.04) intakeRot = -0.04; 
        if (armRot > 2.2) armRot = 2.2; 
        if (armRot < 0) armRot = 0; 




        Logger.recordOutput("Final Component Poses", new Pose3d[] { 
        // intake mount
        new Pose3d(
            0 , 0, 0, new Rotation3d(0,0,0)
        ),
        // intake 
        // pitch max -2.4
        // pitch min -.04
        new Pose3d(
            0.22 , 0, 0.11, new Rotation3d(0,intakeRot,0)
        ),
        // outtake mount
        new Pose3d(
            0 , 0,   0 + innerZ, new Rotation3d(0,0,0)
        ),
        // outtake
        // mmax roll 2.2
        // min roll 0
        new Pose3d(
            -0.05 , 0, 0.8 + innerZ, new Rotation3d(armRot,0,0)
        ),
        // base
        // max z 0
        // min z 0
        new Pose3d(
            0 , 0, 0, new Rotation3d(0,0,0)
        ),
        // 2nd
        // max z .84
        // min z 0
        new Pose3d(
            0 , 0, outerZ, new Rotation3d(0,0,0)
        ),
        // inner
        // max z .96
        // min z 0
        new Pose3d(
            0 , 0, innerZ, new Rotation3d(0,0,0)
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
