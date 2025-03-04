 package frc.robot;

import javax.imageio.plugins.tiff.FaxTIFFTagSet;

import org.w3c.dom.html.HTMLAnchorElement;

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
        public static final double kDriveVelocityFactor = kDrivePositionFactor/60; 
        public static final double kPTurning = 0.4; 
        public static final double kDTurning = 0.0; 
        public static final double slowModeMultiplier = 0.5;           
    }

    // Module for Swerve Drive. 
    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.5);

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(29.5);
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            // front left
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
            // front right
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
            // back left
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), 
            // back right
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 2; // 3
        public static final int kBackLeftDriveMotorPort = 8; // 5
        public static final int kBackRightDriveMotorPort = 6; // 7
        public static final int kFrontRightDriveMotorPort = 4; // 1

        public static final int kFrontLeftTurningMotorPort = 1; // 4
        public static final int kBackLeftTurningMotorPort = 7; // 6
        public static final int kBackRightTurningMotorPort = 5;//  8
        public static final int kFrontRightTurningMotorPort = 3; // 2

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9; // 10
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10; // 11
        public static final int kBackRightDriveAbsoluteEncoderPort = 12; // 9

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.246970906849588;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.118271994368632;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.733242816609336;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -3.015806228983171; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond/2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;   
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }


    // Module for Autonomous Mode. 
    public static final class AutoConstants {
        public static final PPHolonomicDriveController ppConfig = new PPHolonomicDriveController(
        // test .137 
        new PIDConstants(0,0,0), 
        // .137
        new PIDConstants( 0, 0, 0)
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

    public static final class IntakeConstants { 
        public static final int angleMotorID = 14; 
        public static final int intakeMotorID = 11; 

        public static final double groundPosition = 0; 
        public static final double feedPosition = 0; 

        public static final double angleP = 0; 
        public static final double angleI = 0; 
        public static final double angleD = 0; 
        
    }

    public static final class DeliveryConstants { 
        public static final int armMotorID = 15; 
        
        public static final int gripperReverseChannel = 7; 
        public static final int pneumaticHubID = 0; 

        public static final double restPosition = 42; 
        public static final double level1Position = 12; 
        public static final double level2Position = 13; 
        public static final double level3Position = 18; 
        public static final double level4Position = 18; 
        public static final double maxPosition = 0; 
        public static final double minPosition = 0; 

        public static final double armP = 0.09;
        public static final double armI = 0; 
        public static final double armD = 0; 
    }

    public static final class ElevatorConstants { 
        public static final int leaderMotorID = 13; 
        public static final int followerMotorID = 10;
        
        public static final double restPosition = 102.97858428955078; 
        public static final double level1Position = 0; 
        public static final double level2Position = 0; 
        public static final double level3Position = 0; 
        public static final double level4Position = 145; 
 
        public static final double elevatorP = 0.03; 
        public static final double elevatorI = 0; 
        public static final double elevatorD = 0;
    }

    public static final class LimelightConstants { 
        public static final double limelightP = 0; 
        public static final double limelightI = 0; 
        public static final double limelightD = 0; 
    }

    public static final class photonVisionConstants {
        public static final double driveP = 0; 
        public static final double driveI = 0; 
        public static final double driveD = 0;  
        public static final double rotP = 0; 
        public static final double rotI = 0; 
        public static final double rotD = 0; 
    }

}
