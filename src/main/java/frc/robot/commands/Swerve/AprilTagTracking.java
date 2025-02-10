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

    public AprilTagTracking(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; 
        addRequirements(swerveSubsystem);
    }


    @Override
    public void execute() {
        try { 
            if(swerveSubsystem.hasAprilTagTarget()) {
                SmartDashboard.putNumber("APRILTAGX", swerveSubsystem.getAprilTagX(22)); 
                double xSpeed = translationController.calculate(swerveSubsystem.getAprilTagY(22), 1);
                SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                SmartDashboard.putNumber("x speed", xSpeed);
                // xSpeed = limiter.calculate(xSpeed); 
    
                // xSpeed = limiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    
                // if(xSpeed > 1 && xSpeed != 0) { 
                //     xSpeed = 1; 
                // } else if(xSpeed < 1 && xSpeed !=0) { 
                //     xSpeed = -1; 
                // }
    
    
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, 0, 0, swerveSubsystem.getRotation2d()); 
        
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
