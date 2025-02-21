package frc.robot.Subsystems.Swerve.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private SparkMax leaderMotor = new SparkMax(Constants.ElevatorConstants.leaderMotorID, MotorType.kBrushless); 
    private SparkMax followerMotor = new SparkMax(Constants.ElevatorConstants.followerMotorID, MotorType.kBrushless); 

    private RelativeEncoder elevatorEncoder; 
    private SparkClosedLoopController elevatorController; 

    private SparkMaxConfig leaderConfig = new SparkMaxConfig(); 
    private SparkMaxConfig followerConfig = new SparkMaxConfig(); 
    private SparkMaxConfig globalConfig = new SparkMaxConfig();

    public Elevator() { 
        elevatorEncoder = leaderMotor.getEncoder(); 
        elevatorController = leaderMotor.getClosedLoopController(); 

        globalConfig
            .idleMode(IdleMode.kBrake);
        
        leaderConfig
            .apply(globalConfig); 
        
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-.7, .7)
            .pid(Constants.ElevatorConstants.elevatorP, Constants.ElevatorConstants.elevatorI, Constants.ElevatorConstants.elevatorD);  

        followerConfig
            .apply(globalConfig)
            .follow(leaderMotor, true);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    }

    


    
}
