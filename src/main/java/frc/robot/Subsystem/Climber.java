package frc.robot.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private SparkMax leaderMotor = new SparkMax(Constants.leaderMotorID, MotorType.kBrushless); 
    private SparkMax followerMotor = new SparkMax(Constants.followerMotorID, MotorType.kBrushless); 

    public Climber() { 
        SparkMaxConfig config = new SparkMaxConfig(); 

        config.follow(leaderMotor, true); 

        followerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        leaderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveUp() { 
        leaderMotor.set(.5);
    }

    public void stop() { 
        leaderMotor.set(0);
    }

    public void moveDown() { 
        leaderMotor.set(-.5);
    }




    
}
