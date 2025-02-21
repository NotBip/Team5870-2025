package frc.robot.Subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    // Initialize Motors
    private SparkMax angleMotor = new SparkMax(Constants.IntakeConstants.angleMotorID, MotorType.kBrushless); 
    private SparkMax intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless); 

    // Define Relative Encoder. 
    private RelativeEncoder angleEncoder; 

    // Define PID Controller. 
    private SparkClosedLoopController angleController; 

    // Initialize SparkMax Config
    private SparkMaxConfig m_Config = new SparkMaxConfig(); 

    public Intake() { 
        angleEncoder = intakeMotor.getEncoder(); 
        angleController = angleMotor.getClosedLoopController(); 

        m_Config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        m_Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-.5, .5)
            .pid(Constants.IntakeConstants.angleP, Constants.IntakeConstants.angleI, Constants.IntakeConstants.angleD);

        angleMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    }

    public void rotateIntakeForward() { 
        angleMotor.set(.2);
    }

    public void rotateIntakeBack() { 
        angleMotor.set(-.2);
    }

    public void spinWheels(double speed) { 
        intakeMotor.set(speed);
    }

    public void setPosition(double position) { 
        angleController.setReference(position, ControlType.kPosition); 
    }

    public void setAnglePID(double P, double I, double D) { 
        m_Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(P, I, D); 
        
        angleMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("PID HAS BEEN SET");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Intake Encoder", angleEncoder.getPosition()); 
    }

}
