package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    // Initializing both the Intake Motors (Victor SP motors). 
    private VictorSP intakeFront = new VictorSP(IntakeConstants.armMotor1); 
    private VictorSP intakeBack = new VictorSP(IntakeConstants.armMotor2);

    /**
     * This method is used to spin the front motor of the intake. 
     * @param speed The speed at which the motor spins. (-1 to 1)
     */
    public void spinFront(double speed) { 
        intakeFront.set(speed);
    }

    /**
     * This method is used to spin the back motor of the intake. 
     * @param speed The speed at which the motor spins. (-1 to 1)
     */
    public void spinBack(double speed) { 
        intakeBack.set(speed);
    }

    /**
     * Method used to stop both the intake motors. Should be the default motor command. 
     */
    public void stopMotors() { 
        intakeFront.set(0);
        intakeBack.set(0);
    }

    
}
