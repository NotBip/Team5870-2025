package frc.robot.commands.AutoCommands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class MoveLeft extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private boolean isDone; 
    private PIDController transController = new PIDController(photonVisionConstants.driveP, photonVisionConstants.driveI, photonVisionConstants.driveD);
        private PhotonPipelineResult results; 



    public MoveLeft(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; 
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        isDone = false; 
        swerveSubsystem.resetOdometry(new Pose2d());
        swerveSubsystem.zeroHeading();
    }

    @Override
    public void execute() {

        results = swerveSubsystem.getSourceResults(); 

        double xSpeed = 0; 
        double ySpeed = 0;
        double rotSpeed = 0;  

        ySpeed = transController.calculate(swerveSubsystem.getPose().getY(), 1.6); 
        if(swerveSubsystem.getPose().getY() > 1.4) { 
            xSpeed = transController.calculate(swerveSubsystem.getPose().getX(), -2); 
            rotSpeed = 0; 
        }    

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, 1.5, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if(swerveSubsystem.getPose().getX() < -1.7 || swerveSubsystem.findID(results, 12)) { 
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
