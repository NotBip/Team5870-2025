package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
    
    // Initializing both the Climber Motors (CANSparkMax). Look at CANSparkMax api for more info...
    private CANSparkMax leaderMotor; 
    private CANSparkMax followerMotor;
    
    // Constructor runs ONLY ONCE everytime this class in initalized
    public Climber() { 
        
        // Assigning the motors to the variables. 
        leaderMotor = new CANSparkMax(climberConstants.leaderMotor, MotorType.kBrushless);
        followerMotor = new CANSparkMax(climberConstants.followerMotor, MotorType.kBrushless); 
        
        // Resetting both the motors to clear any faults. 
        leaderMotor.restoreFactoryDefaults(); 
        followerMotor.restoreFactoryDefaults(); 

        // Inverting the follower motor and having it follow the Leader. 
        followerMotor.follow(leaderMotor, true);    // Inverting the motor makes it so it spins the opposite direction of the leader motor. 
    }

    /**
     * Method to be used when moving the climber up.
     * @param speed The speed at which the climber moves up (0 to 1). (Any value below 0 is changed to positive)
    */
    public void climberUp(double speed) { 
        leaderMotor.set(Math.abs(speed));
    }

    /**
     * Method to be used when moving the climber down.
     * @param speed The speed at which the climber moves down (-1 to 0). (Any value above 0 is changed to negative)
    */
    public void climberDown(double speed) { 
        leaderMotor.set(-(Math.abs(speed)));
    }

    /**
     * Stops the climber from moving and having it on brake modes. Should be used as the default command for Climber. 
    */
    public void climberStop() { 
        leaderMotor.set(0); 
        leaderMotor.setIdleMode(IdleMode.kBrake); 
        followerMotor.setIdleMode(IdleMode.kBrake); 
    }

}
