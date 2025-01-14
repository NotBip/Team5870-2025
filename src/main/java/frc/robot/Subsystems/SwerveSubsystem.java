package frc.robot.Subsystems;


import java.sql.Driver;
import java.sql.Struct;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {
    
    private AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    public SwerveModule[] SwerveMods;
    private SwerveDriveOdometry odometer; 

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                 zeroHeading();
            } catch (Exception e){}
        }).start();

            SwerveMods = new SwerveModule[] {
                new SwerveModule(
                    0,
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveEncoderReversed,
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, 
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort),
                    
                new SwerveModule(
                    1,
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort),

                new SwerveModule(
                    2,
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveEncoderReversed,
                    DriveConstants.kBackLeftTurningEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort),

                new SwerveModule(
                    3,
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveEncoderReversed,
                    DriveConstants.kBackRightTurningEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort),

        };

        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());

        RobotConfig config;

        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          config = null;
          e.printStackTrace();
        }
            AutoBuilder.configure(
                this::getPose, 
                this::resetOdometry, 
                this::getSpeeds, 
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards, 
                AutoConstants.ppConfig, 
                config, 
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
              );
    }



    public void zeroHeading() {
        navx.reset(); 
    }


    public double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());

    }

    public Pose2d getPose() { 
        return odometer.getPoseMeters(); 
    }

    public void resetOdometry(Pose2d pose) { 
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        getAbsoluteEncoder();

        // SmartDashboard.putData("Set PID", new InstantCommand(() -> reconfigureAuto())); 
    }

    // private void reconfigureAuto() { 
        
    //     double driveP = SmartDashboard.getNumber("Drive P: ", 0.0); 
    //     double driveI = SmartDashboard.getNumber("Drive I: ", 0.0); 
    //     double driveD = SmartDashboard.getNumber("Drive D: ", 0.0); 

    //     double angleP = SmartDashboard.getNumber("Turning P: ", 0.0); 
    //     double angleI = SmartDashboard.getNumber("Turning I: ", 0.0); 
    //     double angleD = SmartDashboard.getNumber("Turning D: ", 0.0); 

    //     AutoBuilder.configureHolonomic(
    //         this::getPose, 
    //         this::resetOdometry, 
    //         this::getSpeeds, 
    //         this::driveRobotRelative, 
    //      new HolonomicPathFollowerConfig(
    //         new PIDConstants(driveP, driveI, driveD),
    //         new PIDConstants(angleP, angleI, angleD),
    //         5, 
    //         0.682498, // Drive base radius (distance from center to furthest module) 
    //         new ReplanningConfig()), 
    //                    () -> { 
    //             var alliance = DriverStation.getAlliance(); 
    //             if(alliance.get() == DriverStation.Alliance.Red)
    //                 return true; 
    //             else
    //                 return false;
    //         }, this);    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : SwerveMods){
            positions[mod.modNum] = mod.getPositions();
        }
        return positions;
    }

    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] positions = new SwerveModuleState[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getState();
        }
        return positions;
    }


    public void stopModules() {
        SwerveMods[0].stop();
        SwerveMods[1].stop();
        SwerveMods[2].stop();
        SwerveMods[3].stop();
    }


    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveMods[0].setDesiredState(desiredStates[0], "Front Left");
        SwerveMods[1].setDesiredState(desiredStates[1], "Front Right");
        SwerveMods[2].setDesiredState(desiredStates[2], "Back Left");
        SwerveMods[3].setDesiredState(desiredStates[3], "Back Right");
        SmartDashboard.putNumber("Wheel Speeds", desiredStates[0].speedMetersPerSecond); 
        SmartDashboard.putNumber("Wheel Angle", desiredStates[0].angle.getDegrees()); 

    }

    public void getAbsoluteEncoder() { 
        SmartDashboard.putNumber("Front Left", SwerveMods[0].getTurningPosition());
        SmartDashboard.putNumber("Front RIght", SwerveMods[1].getTurningPosition());
        SmartDashboard.putNumber("Back Left", SwerveMods[2].getTurningPosition());
        SmartDashboard.putNumber("Back Right", SwerveMods[3].getTurningPosition());
    }

    public ChassisSpeeds getSpeeds() { 
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }



} // end Class
 
