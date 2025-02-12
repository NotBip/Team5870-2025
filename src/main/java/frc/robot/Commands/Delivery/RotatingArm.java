package frc.robot.Commands.Delivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.Delivery;

public class RotatingArm extends Command  {


    private Delivery delivery;

    public RotatingArm (Delivery delivery){
        this.delivery = delivery;
        addRequirements(delivery);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        delivery.RotatingArm();
    }
    


}
