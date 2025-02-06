package frc.robot;

import org.opencv.core.Mat;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // Module for Each Swerve Module.
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 6.75;
        public static final double kTurningMotorGearRatio = (150.0/7); 
        public static final double TurnpositionConversionFactor = (1 / kTurningMotorGearRatio) * Math.PI * 2;
        public static final double TurnVelocityConversionFactor = TurnpositionConversionFactor / 60;
        public static final double kDrivePositionFactor = (1 / kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters);
        // public static final double kDrivePositionFactor = Units.inchesToMeters(4) * Math.PI * kDriveMotorGearRatio;
        public static final double kDriveVelocityFactor = kDrivePositionFactor/60; 
        public static final double kPTurning = 0.4; 
        public static final double kDTurning = 0.0; 
        public static final double slowModeMultiplier = 0.5;           
    }
        
    // Module for Intake Constants
    public static final class IntakeConstants { 
        public static double intakeSpeed = 0.5; 
        public static int armMotor1 = 9; 
        public static int armMotor2 = 8; 
    }

    public static final class PneumaticsConstants { 
        public static int solenoidExtend1ID = 0; 
        public static int solenoidExtend2ID = 2; 
        public static int solenoidDetract1ID = 1; 
        public static int solenoidDetract2ID = 4; 
        public static int compressorID = 0; 
    }

    // Module for Swerve Drive. 
    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(13.5);

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(13.5);
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            // front left
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
            // front right
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
            // back left
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), 
            // back right
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 3; // 3
        public static final int kBackLeftDriveMotorPort = 5; // 5
        public static final int kBackRightDriveMotorPort = 7; // 7
        public static final int kFrontRightDriveMotorPort = 1; // 1

        public static final int kFrontLeftTurningMotorPort = 4; // 4
        public static final int kBackLeftTurningMotorPort = 6; // 6
        public static final int kBackRightTurningMotorPort = 8;//  8
        public static final int kFrontRightTurningMotorPort = 2; // 2

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 9;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.307107104980004;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.52017496079467;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.954291523766307;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.88234990043712; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond/2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }


    public static final double transP = 0; 
    public static final double transI = 0; 
    public static final double transD = 0; 
    public static final double rotP = 0; 
    public static final double rotI = 0; 
    public static final double rotD = 0; 

    
    // Module for Autonomous Mode. 
    public static final class AutoConstants {
        public static final PPHolonomicDriveController ppConfig = new PPHolonomicDriveController(
        // test .137 
        new PIDConstants(.0197,0,.0095), 
        // .137
        new PIDConstants( 1.5, 0, 0)
        ); 
    }


    // Module for Controller Joystick
    public static final class OIConstants {

        // Joystick Values Used for Swerve Controls
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1; 
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final double kDeadband = 0.1;

        // Xbox Controller Map
        public static final int KXboxButtonA = 1; 
        public static final int KXboxButtonB = 2;
        public static final int KXboxButtonX = 3;  
        public static final int KXboxButtonY = 4; 
        public static final int KXboxLeftBumper = 5; 
        public static final int KXboxRightBumper = 6; 
        public static final int KXboxSelectButton = 7; 
        public static final int KXboxStartButton = 8; 
        public static final int KXboxLeftTrigger = 9;
        public static final int KXboxRightTrigger = 10; 
        
    }

    public static final class climberConstants { 
        public static final int leaderMotor = 14; 
        public static final int followerMotor = 13;
        public static final int bottomLimitSwitch = 0; 
        public static final int topLimitSwitch = 0;  
    } 
}
