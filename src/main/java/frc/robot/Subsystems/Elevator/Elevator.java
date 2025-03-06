package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            .outputRange(-1, 1)
            .pid(Constants.ElevatorConstants.elevatorP, Constants.ElevatorConstants.elevatorI, Constants.ElevatorConstants.elevatorD);  

        followerConfig
            .apply(globalConfig)
            .follow(leaderMotor, true);

        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    }

    public void elevatorUp(double speed) { 
        // 170
        // temp 160
        if(elevatorEncoder.getPosition() > 160){
            leaderMotor.set(-.5);
        } else { 
            leaderMotor.set(Math.abs(speed)); 
        }
    }

    public void elevatorDown(double speed) { 
        if(elevatorEncoder.getPosition() < 0) { 
            leaderMotor.set(.5);
        } else { 
            leaderMotor.set(-(Math.abs(speed)));
        }
    }

    public void stopElevator() { 
        leaderMotor.set(0);
    }

    public void setPoint(double position) { 
        System.out.println(position);
        elevatorController.setReference(position, ControlType.kPosition);
    }

    public double getElevatorEncoder() { 
        return elevatorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getPosition()); 
    }

    public void setElevatorPID(double P, double I, double D) { 
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(P, I, D); 
    }


    


    
}
