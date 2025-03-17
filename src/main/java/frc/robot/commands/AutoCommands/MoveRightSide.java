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

public class MoveRightSide extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private boolean isDone; 
    private PIDController transController = new PIDController(photonVisionConstants.driveP, photonVisionConstants.driveI, photonVisionConstants.driveD);
    private PIDController rotController = new PIDController(photonVisionConstants.rotP, photonVisionConstants.rotI, photonVisionConstants.rotD);
    private PhotonPipelineResult results; 


    public MoveRightSide(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; 
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        isDone = false; 
        swerveSubsystem.resetOdometry(new Pose2d());
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        double xSpeed = 0; 
        double rotSpeed = 0; 

        results = swerveSubsystem.getSourceResults(); 

        xSpeed = transController.calculate(swerveSubsystem.getPose().getX(), -1); 

        if(swerveSubsystem.getPose().getX() < -0.6) { 
            rotSpeed = 1.5; 
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-xSpeed, 0, rotSpeed);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        if(swerveSubsystem.findID(results, 13)) { 
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
