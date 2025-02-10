package frc.robot.commands.Swerve;

import javax.xml.xpath.XPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class AprilTagTracking extends Command {

    private SwerveSubsystem swerveSubsystem;
    PIDController translationController = new PIDController(Constants.transP, Constants.transI, Constants.transD); 
    PIDController rotationalController = new PIDController(Constants.rotP, Constants.rotI, Constants.rotD);
    int trackerID; 


    public AprilTagTracking(SwerveSubsystem swerveSubsystem, int trackerID) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.trackerID = trackerID; 
        addRequirements(swerveSubsystem);
    }


    @Override
    public void execute() {
        try { 
            if(swerveSubsystem.hasAprilTagTarget()) {

                double xDist = swerveSubsystem.getAprilTagX(trackerID); 
                double yDist = swerveSubsystem.getAprilTagY(trackerID);

                double xSpeed = translationController.calculate(xDist, 1);
                double ySpeed = translationController.calculate(yDist, 0); 
    
                
                // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, 0, swerveSubsystem.getRotation2d()); 
                
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, -xSpeed, 0);

                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                swerveSubsystem.setModuleStates(moduleStates);   
    
            } else { 
                swerveSubsystem.stopModules();
            }
        } catch (Exception e) { 
            e.printStackTrace();
        }
        
    }
    
}
