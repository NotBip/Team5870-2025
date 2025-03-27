package frc.robot.commands.AprilTagAlignmentCommands;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class AutoAlignSwerve extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private PIDController driveController = new PIDController(Constants.photonVisionConstants.driveP, Constants.photonVisionConstants.driveI, Constants.photonVisionConstants.driveD); 
    private PIDController rotContoller = new PIDController(photonVisionConstants.rotP, photonVisionConstants.rotI, photonVisionConstants.rotD);
    private PhotonPipelineResult results;
    private boolean initialAlignment; 
    private boolean rightSide;
    private boolean isDone;
    double ySetpoint; 

    public AutoAlignSwerve(SwerveSubsystem swerveSubsystem, boolean rightSide) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.rightSide = rightSide;
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() { 
        rotContoller.enableContinuousInput(-180,180);
        initialAlignment = false; 
        isDone = false;
        if(rightSide) { 
            ySetpoint = -.15; 
        } else { 
            ySetpoint = -.48; 
        }
    }
    
    @Override
    public void execute() {
        results = swerveSubsystem.getReefResults(); 
        PhotonTrackedTarget target = results.getBestTarget();


        if(target != null && initialAlignment == false) { 
            double xDist = swerveSubsystem.getAprilTagX(target);
            double yDist = swerveSubsystem.getAprilTagY(target);
            double rotDist = swerveSubsystem.getPhotonAprilTagTheta(target.getFiducialId(), results); 
             
            double xSpeed = driveController.calculate(xDist, .9);
            double ySpeed = driveController.calculate(yDist, ySetpoint); 
            double rotSpeed = rotContoller.calculate(rotDist, 175);

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -rotSpeed);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);

            if((xDist <= 1.1 && xDist >= .7) && (yDist <=   ySetpoint + .04 && yDist >= ySetpoint - .04)) { 
                swerveSubsystem.resetOdometry(new Pose2d()); 
                initialAlignment = true; 
            }
        }

        if(initialAlignment == true) { 
            double xDist = Math.abs(swerveSubsystem.getPose().getX()); 
            double xSpeed = driveController.calculate(xDist, 1.3);

            
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, 0);
            
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);
            SmartDashboard.putNumber("x dist", xDist);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(initialPose.getTranslation().getX() + swerveSubsystem.getPose().getTranslation().getX(), swerveSubsystem.getPose().getTranslation().getY()), swerveSubsystem.getPose().getRotation()));
    }



    @Override
    public boolean isFinished() {
        return false; 
    }

    
}
