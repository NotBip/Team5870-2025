package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant;
import frc.robot.Constant.*;



public class Elevator extends SubsystemBase {
    
    private SparkMax leaderMotor;
    private SparkMax followingMotor;

    private SparkMaxConfig m_Config;


    public Elevator(){
        leaderMotor = new SparkMax(Constant.leaderMotorID, MotorType.kBrushless);
        followingMotor = new SparkMax(Constant.followingMotorID, MotorType.kBrushless);
        m_Config = new SparkMaxConfig();
    
        m_Config.follow(leaderMotor, true);

        m_Config.closedLoop.pid(0,0,0); 
    

    }

    public void moveMotorUp(){
        System.out.println("Elevator moving");
        leaderMotor.set(0.5);
    }

    public void moveMotorDown(){
        leaderMotor.set(-0.5);
    }

    public void stopMotor(){
        // SmartDashboard.putBoolean("MOVING", false);
        leaderMotor.set(0);
    }

}
