package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private VictorSP intakeFront = new VictorSP(IntakeConstants.armMotor1); 
    private VictorSP intakeBack = new VictorSP(IntakeConstants.armMotor2);

    public void spinFront(double speed) { 
        intakeFront.set(speed);
    }

    public void spinBack(double speed) { 
        intakeBack.set(speed);
    }

    public void stopMotors() { 
        intakeFront.set(0);
        intakeBack.set(0);
    }

    
}
