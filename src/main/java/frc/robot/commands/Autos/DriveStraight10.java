package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.SwerveSubsystem;

/**
 * This class is used to generate a command for driving straight for 10 metres. 
 */
public class DriveStraight10 extends SequentialCommandGroup{

    public DriveStraight10(SwerveSubsystem swerveSubsystem) { 
            
        Pose2d initialPose; // Define the initial robot pose. 

        // Flip the iniial pose from the path file if red alliance. 
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { 
            initialPose = PathPlannerPath.fromPathFile("Straight10").flipPath().getPreviewStartingHolonomicPose();
        } else { 
            initialPose = PathPlannerPath.fromPathFile("Straight10").getPreviewStartingHolonomicPose(); 
        }

        // Commands to reset odometry and follow path. 
        addCommands(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose)), 
            new InstantCommand(() -> AutoBuilder.followPath(PathPlannerPath.fromPathFile("Straight10"))) 
        );
    }

}
