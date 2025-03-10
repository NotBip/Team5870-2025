package frc.robot.commands.Swerve;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class RedAutoAlignReef extends Command {
    
    private SwerveSubsystem swerveSubsystem;
    private Arm arm;
    private Elevator elevator;  

    public RedAutoAlignReef(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) { 
        this.swerveSubsystem = swerveSubsystem; 
        this.arm = arm; 
        this.elevator = elevator; 
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        arm.setPosition(Constants.DeliveryConstants.level1Position);
        if(arm.getArmEncoder() < Constants.DeliveryConstants.level1Position + 10.0) { 
            elevator.setPoint(Constants.ElevatorConstants.level1Position);
        }
    }



    @Override
    public void end(boolean interrupted) {
        arm.openGripper();
        arm.armStop();
        elevator.stopElevator();
    }


    @Override
    public boolean isFinished() {
        return false; 
    }




    
}
