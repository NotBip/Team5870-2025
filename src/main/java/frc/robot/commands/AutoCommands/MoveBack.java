package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class MoveBack extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private boolean isDone; 
    private PIDController transController = new PIDController(photonVisionConstants.driveP, photonVisionConstants.driveI, photonVisionConstants.driveD);
    private double dist; 


    public MoveBack(SwerveSubsystem swerveSubsystem, double dist) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.dist = dist; 
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        isDone = false; 
        swerveSubsystem.resetOdometry(new Pose2d());
    }

    @Override
    public void execute() {

        double xSpeed = 0; 

        xSpeed = transController.calculate(swerveSubsystem.getPose().getX(), dist);   

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-xSpeed*2, 0, 0);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if(swerveSubsystem.getPose().getX() > dist/2) { 
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
