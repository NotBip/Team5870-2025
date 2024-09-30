package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
    
    private CANSparkMax leaderMotor; 
    private CANSparkMax followerMotor;
    
    public Climber() { 
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless); 
        
        leaderMotor.restoreFactoryDefaults(); 
        followerMotor.restoreFactoryDefaults(); 

        followerMotor.follow(leaderMotor, true);
    }

    public void climberUp(double speed) { 
        leaderMotor.set(speed);
    }

    public void climberDown(double speed) { 
        leaderMotor.set(speed);
    }

    public void climberStop() { 
        leaderMotor.set(0);
        leaderMotor.setIdleMode(IdleMode.kBrake); 
        followerMotor.setIdleMode(IdleMode.kBrake); 
    }

}
