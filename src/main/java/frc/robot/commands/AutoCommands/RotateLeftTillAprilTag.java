package frc.robot.commands.AutoCommands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class RotateLeftTillAprilTag extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private boolean isDone; 
    private PhotonPipelineResult results; 
    private int ID; 
    private PIDController transController = new PIDController(photonVisionConstants.driveP, photonVisionConstants.driveI, photonVisionConstants.driveD);
    
    public RotateLeftTillAprilTag(SwerveSubsystem swerveSubsystem, int ID) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.ID = ID; 
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        isDone = false; 
        swerveSubsystem.resetOdometry(new Pose2d());
    }

    @Override
    public void execute() {

        results = swerveSubsystem.getSourceResults(); 

        double xSpeed = 0; 


        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1.5, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if(swerveSubsystem.findID(results, ID)) { 
            isDone = true; 
        }
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        return isDone; 
    }
    
}
