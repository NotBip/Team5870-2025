package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

        double xSpeed = translationController.calculate(swerveSubsystem.getAprilTagX(12), 5);
        double ySpeed = translationController.calculate(swerveSubsystem.getAprilTagY(12), 5);
        // double rotSpeed = rotationalController.calculate(swerveSubsystem.getHeading(), swerveSubsystem.getAprilTagRot(12));
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, swerveSubsystem.getRotation2d()); 

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }
    
}
