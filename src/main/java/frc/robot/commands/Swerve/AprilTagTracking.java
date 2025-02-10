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
                // SmartDashboard.putNumber("APRILTAGX", swerveSubsystem.getAprilTagX(22)); 
                double xDist = swerveSubsystem.getAprilTagX(22); 
                double yDist = swerveSubsystem.getAprilTagY(22);
                double xNormalized = 0; 
                double yNormalized = 0; 
                // System.out.println(xDist);

                // 3 Max Dist
                // 0.3 is min Dist
                // if(xDist > 1.05 && xDist < 3) { 
                //     xNormalized = xDist - 1 * (3/2); 
                // } else if (xDist < .95) { 
                //     xNormalized = 3.0 * (xDist - (0.3 / (1.0 -0.3))); 
                // }
                xNormalized = -xDist;  
                // SmartDashboard.putNumber("XDISTAFTER", xDist);

                // 0.6 is Minimum and Max horizontal Dist
                // 3 is Max Speed
                yNormalized = (yDist); 


                double xSpeed = translationController.calculate(xDist, 1);
                double ySpeed = translationController.calculate(yDist, 0); 

                

                // double ySpeed = translationController.calculate(swerveSubsystem.getAprilTagY(22), 0);
                // SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                // SmartDashboard.putNumber("x speed", xSpeed);
                // xSpeed = limiter.calculate(xSpeed); 
    
                // xSpeed = limiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    
                // if(xSpeed > 1 && xSpeed != 0) { 
                //     xSpeed = 1; 
                // } else if(xSpeed < 1 && xSpeed !=0) { 
                //     xSpeed = -1; 
                // }
    
    
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, 0, swerveSubsystem.getRotation2d()); 
        
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
