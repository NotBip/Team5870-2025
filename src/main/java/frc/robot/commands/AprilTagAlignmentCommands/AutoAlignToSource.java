package frc.robot.commands.AprilTagAlignmentCommands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class AutoAlignToSource extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private PIDController driveController = new PIDController(Constants.photonVisionConstants.driveP, Constants.photonVisionConstants.driveI, Constants.photonVisionConstants.driveD); 
    private PIDController rotController = new PIDController(Constants.photonVisionConstants.rotP, Constants.photonVisionConstants.rotI, Constants.photonVisionConstants.rotD); 
    private boolean isRedAlliance, isRightSide; 
    private int trackerID; 
    private PhotonPipelineResult results;
    private boolean initialAlignment = false; 
    private boolean isDone = false; 
    private double initalX; 
    private Pose2d initialPose; 

    public AutoAlignToSource(SwerveSubsystem swerveSubsystem, boolean isRedAlliance, boolean isRightSide) { 
        this.isRightSide = isRightSide;
        this.swerveSubsystem = swerveSubsystem; 
        this.isRedAlliance = isRedAlliance; 
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() { 
        if(isRedAlliance == true) { 
            if(isRightSide == true) { 
                trackerID = 1;  
            } else { 
                trackerID = 2;
            }
        } else { 
            if(isRightSide == true) { 
                trackerID = 13;  
            } else { 
                trackerID = 12;
            }
        }
    }
    
    @Override
    public void execute() {
        results = swerveSubsystem.getSourceResults(); 

        if(swerveSubsystem.hasPhotonAprilTagTarget(results) && initialAlignment == false) { 
            double xDist = swerveSubsystem.getPhotonAprilTagX(trackerID, results);
            double yDist = swerveSubsystem.getPhotonAprilTagY(trackerID, results); 
            double rotDist = swerveSubsystem.getPhotonAprilTagTheta(trackerID, results); 

            double xSpeed = driveController.calculate(xDist, 1.5);
            double ySpeed = driveController.calculate(yDist, 0); 
            double rotSpeed = rotController.calculate(rotDist, 0); 

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
            
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);

            if((xDist <= 1.7 && xDist >= 1.3) && (yDist <= 0.2 && yDist >= -0.2)) { 
                initialPose = swerveSubsystem.getPose(); 
                swerveSubsystem.resetOdometry(new Pose2d()); 
                initialAlignment = true; 
            }
        }

        if(initialAlignment == true) { 
            double xDist = swerveSubsystem.getPose().getTranslation().getX(); 
            double xSpeed = driveController.calculate(xDist, 1.5);
            
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, 0);
            
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);

            if(xDist > 1.3) { 
                isDone = true; 
                swerveSubsystem.stopModules();
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(initialPose.getTranslation().getX() + swerveSubsystem.getPose().getTranslation().getX(), swerveSubsystem.getPose().getTranslation().getY()), swerveSubsystem.getPose().getRotation()));
    }



    @Override
    public boolean isFinished() {
        return isDone; 
    }

    
}
