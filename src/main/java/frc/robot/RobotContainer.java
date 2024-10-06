package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;

public class RobotContainer {
    
    // Initializing Robot's Subsystems
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // Initializing Controllers
    private final CommandXboxController driverController = new CommandXboxController(IOConstants.kDriverControllerPort); 

    // Initializing Commands
    // Swerve
    private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem);  // Resets the gyro on the robot to reconfigure the absolute front of the robot. 
    
    public RobotContainer() {
        
        // Default command for this swerve drive so it always takes in controller input by default and is on standby. 
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverController.getRawAxis(IOConstants.kDriverYAxis),
                () -> driverController.getRawAxis(IOConstants.kDriverXAxis),
                () -> -driverController.getRawAxis(IOConstants.kDriverRotAxis),
                () -> !driverController.rightBumper().getAsBoolean(),          
                () -> driverController.getRightTriggerAxis() > 0.5 ? true : false));     

        configureButtonBindings(); // Bind all the controller buttons to commands. 
    }

    /**
     * This method should always be called at the very end of the constructor in the RobotContainer Class. This method is used to assign driver/operator controller buttons 
     * to commands. 
     */
    private void configureButtonBindings() {
        
        // ======================================================== DRIVER COMMANDS =========================================================================================
        // QOL Swerve Commands
        driverController.start().onTrue(zeroGyro);  // .onTrue only runs the Command once when it's pressed.
    }

    /**
     * Method used to return whichever autonomous routine/command you are running. 
     * @return Returns a autonmous routine chosen from smartDashboard whenever auto mode is enabled. 
     */
    public Command getAutonomousCommand() {                
        return null; 
    }        

    public Command getSelfTestCommand() { 
        return null; 
    }
}