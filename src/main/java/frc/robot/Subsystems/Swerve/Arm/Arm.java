package frc.robot.Subsystems.Swerve.Arm;

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

import edu.wpi.first.wpilibj.PneumaticHub;
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
    
    private PneumaticHub pneumaticHub = new PneumaticHub(Constants.DeliveryConstants.pneumaticHubID);
    private Solenoid closeChannel = new Solenoid(PneumaticsModuleType.REVPH, Constants.DeliveryConstants.gripperForwardChannel);
    private Solenoid openChannel = new Solenoid(PneumaticsModuleType.REVPH, Constants.DeliveryConstants.gripperReverseChannel); 


    public Arm() { 

        armEncoder = armMotor.getEncoder(); 
        armController = armMotor.getClosedLoopController(); 

        m_Config
            .idleMode(IdleMode.kBrake)
            .inverted(false); 
        m_Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-.5,.5)
            .pid(Constants.DeliveryConstants.armP, Constants.DeliveryConstants.armI, Constants.DeliveryConstants.armD); 

        armMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        pneumaticHub.enableCompressorDigital();
    }

    public void rotateArm() { 
        armMotor.set(0.2);
    }

    public void rotateArmReverse() { 
        armMotor.set(-0.2);
    }

    public void closeGripper() { 
        closeChannel.set(true);
        openChannel.set(false);
    }

    public void openGripper() { 
        closeChannel.set(false);
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
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition()); 
    }









}
