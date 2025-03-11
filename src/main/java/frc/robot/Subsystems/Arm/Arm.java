package frc.robot.Subsystems.Arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private SparkMax armMotor = new SparkMax(Constants.DeliveryConstants.armMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_Config = new SparkMaxConfig(); 

    private RelativeEncoder armEncoder; 
    private SparkClosedLoopController armController;

    DigitalInput proxSensor = new DigitalInput(9); 
        
    private Solenoid openChannel = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.DeliveryConstants.gripperReverseChannel); 

    public Arm() { 
        armEncoder = armMotor.getEncoder(); 
        armController = armMotor.getClosedLoopController(); 
        m_Config
            .idleMode(IdleMode.kBrake)
            .inverted(false); 
        m_Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-.4,.4)
            .pid(Constants.DeliveryConstants.armP, Constants.DeliveryConstants.armI, Constants.DeliveryConstants.armD); 

        armMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    }

    public void rotateArm() { 
        if(armEncoder.getPosition() > Constants.DeliveryConstants.grabPosition + .2) { 
            armMotor.set(-.4);
        }
    
        if(armEncoder.getPosition() > 24) { 
            armMotor.set(.05);
        } else { 
            armMotor.set(0.4);
        }
    }

    public double getArmEncoder() { 
        return armEncoder.getPosition();
    }

    public boolean getGripper() { 
        return openChannel.get(); 
    }

    public void rotateArmReverse() { 
        
        if(armEncoder.getPosition() < 0) {
            armMotor.set(.4);
        } else {
            if(armEncoder.getPosition() < 24) { 
                armMotor.set(-.07);
            } else {
                armMotor.set(-.2);
            }
        }
        

    }

    public void closeGripper() { 
        openChannel.set(false);
    }

    public void openGripper() { 
        openChannel.set(true);
    }   

    public void setPosition(double position) { 
        armController.setReference(position, ControlType.kPosition); 
    }   



    public void setArmPID(double P, double I, double D) { 
        m_Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(P, I, D); 
        
        armMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("PID HAS BEEN SET");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Gripper", getGripper());
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition()); 
        SmartDashboard.putBoolean("Prox Sensor", proxSensor.get());
        // if(armEncoder.getPosition() < 16) { 
        //     if(openChannel.get() == true) { 
        //         closeGripper();
        //     }
        // }
    }

    public void armStop() { 
        armMotor.stopMotor();
    }









}
