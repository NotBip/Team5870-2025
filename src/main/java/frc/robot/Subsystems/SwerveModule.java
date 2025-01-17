package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    // Initalize the Motors.
    
    public int modNum; 
    private final SparkMax driveMotor; 
    private final SparkMax turningMotor; 

    // Initialize the Encoders. 
    private final RelativeEncoder driveEncoder; 
    private final RelativeEncoder turningEncoder;

    // Motor Configs
    private final SparkMaxConfig driveConfig = new SparkMaxConfig(); 
    private final SparkMaxConfig turningConfig = new SparkMaxConfig(); 

    // Initialziing PID Controller for turning. 
    PIDController turningPidController;

    private final CANcoder absoluteEncoder; 

    // Initalizing ports for encoder. 
    private final boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;
    public double offset = 0; 


    /**g
     * Constructor for each Swerve Module. 
     * @param driveMotorId  Port for drive Motor 
     * @param turningMotorId    Port for Turning Motor.
     * @param driveMotorReversed    Boolean if Drive Motor is Reversed. 
     * @param turningMotorReversed  Boolean if Turning Motor is Reversed. 
     * @param absoluteEncoderId     Analog Input Port of Absolute Encoder
     * @param absoluteEncoderOffset absolute Encoder offset
     * @param absoluteEncoderReversed   Boolean if Absolute Encoder is Reversed. 
     */
    public SwerveModule(int modNum, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int absoluteEncoderId){

        // Set Absolute Encoder Port. 
        this.modNum = modNum; 
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset; 
        this.absoluteEncoderReversed = absoluteEncoderReversed; 

        this.absoluteEncoder = new CANcoder(absoluteEncoderId);

        // Set drive Motor and turning Motor type and port.
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        

        driveConfig.idleMode(IdleMode.kBrake); 

        // Set Motors inverted if true. 
        driveConfig.inverted(driveMotorReversed); 
        turningConfig.inverted(turningMotorReversed);

        // Convert Encoder values. 
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); 

        driveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);


        turningConfig.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);

        turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // Initialzing PID Controller. 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        var toApply = new CANcoderConfiguration();
        // absoluteEncoder.getConfigurator().apply(toApply);

        // Reset Encoders at the start. 
                // Get encoder values for both drive and turning motors. 
        driveEncoder = driveMotor.getEncoder(); 
        turningEncoder = turningMotor.getEncoder();
        resetEncoders();
    }

   
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() { 
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }


    public double getAbsoluteEncoderRad() { 
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        // angle *= Math.PI / 180; 
        angle -= absoluteEncoderOffsetRad; 
        return angle; 
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad()); 
    }
    
  
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, String wheel) { 
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SparkMax getDriveMotor() { 
        return driveMotor; 
    }

    public SwerveModulePosition getPositions(){ 
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getState().angle);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
