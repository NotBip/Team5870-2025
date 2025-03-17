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

public class RotAlignTest extends Command {

    private SwerveSubsystem swerveSubsystem; 
    private PIDController driveController = new PIDController(Constants.photonVisionConstants.driveP, Constants.photonVisionConstants.driveI, Constants.photonVisionConstants.driveD); 
    private PIDController rotController = new PIDController(photonVisionConstants.rotP, photonVisionConstants.rotI, photonVisionConstants.rotD);
    private PhotonPipelineResult results;
    private boolean initialAlignment; 
    private boolean rightSide;
    private boolean isDone;
    double ySetpoint; 

    public RotAlignTest(SwerveSubsystem swerveSubsystem, boolean rightSide) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.rightSide = rightSide;
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() { 
        initialAlignment = false; 
        isDone = false;
        rotController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void execute() {
        results = swerveSubsystem.getReefResults(); 
        PhotonTrackedTarget target = results.getBestTarget();

        if(target != null && initialAlignment == false) { 
            double yaw = swerveSubsystem.getPhotonAprilTagTheta(target.getFiducialId(), results); 
            double rotSpeed = rotController.calculate(yaw, 180); 
            SmartDashboard.putNumber("Rottttt", yaw);


            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, -rotSpeed);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
            swerveSubsystem.setModuleStates(moduleStates);
        }

    }
    
    @Override
    public void end(boolean interrupted) {
    }



    @Override
    public boolean isFinished() {
        return false; 
    }

    
}
