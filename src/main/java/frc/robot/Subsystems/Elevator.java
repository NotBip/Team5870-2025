package frc.robot.Subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant;
import frc.robot.Constant.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;



public class Elevator extends SubsystemBase {
    
    private SparkMax leaderMotor;
    private SparkMax followingMotor;

    private SparkMaxConfig m_Config;

    
    private SparkClosedLoopController m_controller;



    public Elevator(){
        leaderMotor = new SparkMax(Constant.leaderMotorID, MotorType.kBrushless);
        followingMotor = new SparkMax(Constant.followingMotorID, MotorType.kBrushless);
        m_Config = new SparkMaxConfig();
    
        m_Config.follow(leaderMotor, true);

        m_Config.closedLoop.pid(.05,0,0); 
        m_controller = leaderMotor.getClosedLoopController();

        leaderMotor.configure(m_Config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    }

    public void moveMotorUp(){
        SmartDashboard.putNumber("Elevator", .5);
        leaderMotor.set(0.5);
    }

    public void moveMotorDown(){
        SmartDashboard.putNumber("Elevator", -.5);
        leaderMotor.set(-0.5);
    }

    public void stopMotor(){
        SmartDashboard.putNumber("Elevator", .0);
        leaderMotor.set(0);
    }

    public void Level0() { 
        m_controller.setReference(0, ControlType.kPosition);
        System.out.println("Climber Zeroed");
    }
    public void Level1() { 
        m_controller.setReference(0, ControlType.kPosition);
        System.out.println("Climber Zeroed");
    }

    public void Level2() { 
        m_controller.setReference(0, ControlType.kPosition);
        System.out.println("Climber Zeroed");
    }

    public void Level3() { 
        m_controller.setReference(0, ControlType.kPosition);
        System.out.println("Climber Zeroed");
    }

    public void Level4() { 
        m_controller.setReference(0, ControlType.kPosition);
        System.out.println("Climber Zeroed");
    }

}
