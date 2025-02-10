package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {
    
    private AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    public SwerveModule[] SwerveMods;
    private SwerveDriveOdometry odometer; 
    private SwerveDrivePoseEstimator poseEstimator; 
    // ARDUCAM INIT
    private PhotonCamera photonCamera; 
    private PhotonPipelineResult latestResults; 

    public SwerveSubsystem(){

        photonCamera = new PhotonCamera("photoncam");

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

        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), getPose());

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
        PathfindingCommand.warmupCommand().schedule();  
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

        latestResults=photonCamera.getLatestResult(); 
        odometer.update(getRotation2d(), getModulePositions());
        Logger.recordOutput("Pose", getPose());
        Logger.recordOutput("Mod Positions", getModulePositions());

        getAbsoluteEncoder();
        getTurningEnc();
    }


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

    public void resetPoseEst(Pose2d pose) { 
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void updatePoseEst() { 
            poseEstimator.update(getRotation2d(), getModulePositions());
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
        SmartDashboard.putNumber("Front Left", SwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front RIght", SwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Left", SwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Right", SwerveMods[3].getAbsoluteEncoderRad());
    }

    public void getTurningEnc() { 
        SmartDashboard.putNumber("Front Left Turn", SwerveMods[0].getTurningPosition());
        SmartDashboard.putNumber("Front RIght Turn", SwerveMods[1].getTurningPosition());
        SmartDashboard.putNumber("Back Left Turn ", SwerveMods[2].getTurningPosition());
        SmartDashboard.putNumber("Back Right Turn", SwerveMods[3].getTurningPosition());
    }

    public ChassisSpeeds getSpeeds() { 
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
        ChassisSpeeds newSpeeds = new ChassisSpeeds(-robotRelativeSpeeds.vxMetersPerSecond,- robotRelativeSpeeds.vyMetersPerSecond, -robotRelativeSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(newSpeeds); 
        setModuleStates(states);
    }


    // ARDUCAM STUFF
    public double getAprilTagX(int ID) {
        if (latestResults.hasTargets()) {  
            var results = latestResults.getTargets();
            for (int i = 0; i < results.size(); i++) {
                if (results.get(i).getFiducialId() == ID) {
                return results.get(i).getBestCameraToTarget().getX();
                }
            }
        } else {
            return 0;
        }
        return 0;
    }


    public boolean hasAprilTagTarget() {
        if (latestResults.hasTargets()) { 
            return true;
        }
        return false;
    }

    public double getAprilTagY(int ID) {
        if (latestResults.hasTargets()) { 
            var results = latestResults.getTargets();
            for (int i = 0; i < results.size(); i++) {
                if (results.get(i).getFiducialId() == ID) {
                return results.get(i).getBestCameraToTarget().getY();
                }
            }
        } else {
            return 0;
        }
        return 0;
    }


} // end Class
 
