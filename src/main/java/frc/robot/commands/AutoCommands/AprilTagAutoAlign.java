// package frc.robot.commands.AutoCommands;

// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Subsystems.Swerve.SwerveSubsystem;

// public class AprilTagAutoAlign extends Command {

//     private SwerveSubsystem swerveSubsystem; 
//     private PIDController driveController = new PIDController(Constants.photonVisionConstants.driveP, Constants.photonVisionConstants.driveI, Constants.photonVisionConstants.driveD); 
//     private PIDController rotController = new PIDController(Constants.photonVisionConstants.rotP, Constants.photonVisionConstants.rotI, Constants.photonVisionConstants.rotD); 
//     private int trackerID; 
//     private int xOffset, yOffset, rotOffset, cam; 
//     private PhotonPipelineResult results; 

//     public AprilTagAutoAlign(SwerveSubsystem swerveSubsystem, int trackerID, int xOffset, int yOffset, int rotOffset, int cam) { 
//         this.swerveSubsystem = swerveSubsystem; 
//         this.trackerID = trackerID; 
//         this.xOffset = xOffset; 
//         this.yOffset = yOffset; 
//         this.rotOffset = rotOffset; 
//         addRequirements(swerveSubsystem);
//     }
    
//     @Override
//     public void initialize() {

//         if(cam == 0) { 
//             results = swerveSubsystem.getSourceResults(); 
//         } else if(cam == 1) { 
//             results = swerveSubsystem.getReefResults(); 
//         }
        
    
//     }
    
//     @Override
//     public void execute() {
        
//         if(swerveSubsystem.hasPhotonAprilTagTarget(results)) { 

//             double xDist = swerveSubsystem.getPhotonAprilTagX(trackerID, results);
//             double yDist = swerveSubsystem.getPhotonAprilTagY(trackerID, results); 
//             double rotDist = swerveSubsystem.getPhotonAprilTagTheta(trackerID, results); 

//             double xSpeed = driveController.calculate(xDist, xOffset);
//             double ySpeed = driveController.calculate(yDist, yOffset); 
//             double rotSpeed = rotController.calculate(rotDist, rotOffset); 

//             ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
            
//             SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
//             swerveSubsystem.setModuleStates(moduleStates);
        
//         } else { 
//             swerveSubsystem.stopModules();
//         }

//     }
    
//     @Override
//     public void end(boolean interrupted) {
    
//     }



//     @Override
//     public boolean isFinished() {
//         return false; 
//     }

    
// }
