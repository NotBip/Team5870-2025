
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Delivery extends SubsystemBase{
    private SparkMax RotateArmMoterID = new SparkMax(Constants.RotateArmMoterID, MotorType.kBrushless);
    
    private SparkMaxConfig m_config = new SparkMaxConfig();

    public Delivery(){
    }

    public  void RotatingArm ( ) {
        RotateArmMoterID.set(0.5);
        SmartDashboard.putNumber("RotatingArm", 0.5);
    }

      public void Stop ( ) {
        RotateArmMoterID.set(0);
        SmartDashboard.putNumber("RotatingArm", 0);
    }


}
