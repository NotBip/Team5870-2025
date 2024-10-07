package frc.robot.commands.SelfTest.SwerveTest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class SwerveDriveTest extends Command{

    private SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveDriveTest(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = 1; 
        double ySpeed = 0;
        double turningSpeed = 0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed, turningSpeed, swerveSubsystem.getRotation2d());    
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }



    
}
