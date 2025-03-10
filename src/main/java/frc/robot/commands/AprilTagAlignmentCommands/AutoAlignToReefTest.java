package frc.robot.commands.AprilTagAlignmentCommands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class AutoAlignToReefTest extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private PIDController driveController = new PIDController(Constants.photonVisionConstants.driveP, Constants.photonVisionConstants.driveI, Constants.photonVisionConstants.driveD); 
    private PIDController rotController = new PIDController(Constants.photonVisionConstants.rotP, Constants.photonVisionConstants.rotI, Constants.photonVisionConstants.rotD); 
    private int trackerID; 
    private PhotonPipelineResult results;
    private boolean initialAlignment; 
    private boolean isDone; 
    private boolean rightSide;
    double ySetpoint; 
    double xSetpoint; 

    public AutoAlignToReefTest(SwerveSubsystem swerveSubsystem, int trackerID, boolean rightSide) { 
        this.trackerID = trackerID; 
        this.swerveSubsystem = swerveSubsystem; 
        this.rightSide = rightSide;
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() { 
        initialAlignment = false; 
        isDone = false; 
        if(rightSide) { 
            ySetpoint = -.12; 
            xSetpoint = .9; 
        } else { 
            ySetpoint = -.52;
            xSetpoint = 1.5;  
        }
    }
    
    @Override
    public void execute() {
        results = swerveSubsystem.getReefResults(); 

        if(swerveSubsystem.hasPhotonAprilTagTarget(results) && initialAlignment == false) { 
            double xDist = swerveSubsystem.getPhotonAprilTagX(trackerID, results);
            double yDist = swerveSubsystem.getPhotonAprilTagY(trackerID, results);
             
            double xSpeed = driveController.calculate(xDist, xSetpoint);
            double ySpeed = driveController.calculate(yDist, ySetpoint); 

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, 0);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);

            if((xDist <= xSetpoint + .2 && xDist >= xSetpoint - .2) && (yDist <=   ySetpoint + .05 && yDist >= ySetpoint - .05)) { 
                swerveSubsystem.resetOdometry(new Pose2d()); 
                initialAlignment = true; 
            }
        }

        if(initialAlignment == true) { 
            double xDist = Math.abs(swerveSubsystem.getPose().getX()); 
            double xSpeed = driveController.calculate(xDist, xSetpoint + .4);

            
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, 0);
            
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);
            SmartDashboard.putNumber("x dist", xDist);

            if(xDist >= xSetpoint - 0.43) { 
                isDone = true; 
                swerveSubsystem.stopModules();
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(initialPose.getTranslation().getX() + swerveSubsystem.getPose().getTranslation().getX(), swerveSubsystem.getPose().getTranslation().getY()), swerveSubsystem.getPose().getRotation()));
    }



    @Override
    public boolean isFinished() {
        return isDone; 
    }

    
}
