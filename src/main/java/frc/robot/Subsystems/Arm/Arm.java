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

import edu.wpi.first.wpilibj.Compressor;
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
    
    private Solenoid openChannel = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.DeliveryConstants.gripperReverseChannel); 
    private Compressor c; 

    public Arm() { 
       // c =  new Compressor(PneumaticsModuleType.CTREPCM); 
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

        //c.enableDigital();
        // pneumaticHub.enableCompressorDigital();
    }

    public void rotateArm() { 
        armMotor.set(0.2);
    }

    public void rotateArmReverse() { 
        if(armEncoder.getPosition() < 0){
            armMotor.set(.3);
        }
        else{
            //arm rotation encoder values for specific hights
            //0
            //42
            //
            //
            //
        armMotor.set(-0.2);
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
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition()); 
    }

    public void armStop() { 
        armMotor.stopMotor();
    }









}
