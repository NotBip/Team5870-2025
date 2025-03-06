package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class TestAuto2 extends SequentialCommandGroup {
    Pose2d initialPose = new Pose2d(); 

    public TestAuto2(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator, boolean isRedAlliance) { 
        try { 


            initialPose = PathPlannerPath.fromPathFile("TestPath").getStartingHolonomicPose().get(); 
            addCommands(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose)), 
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestPath"))
            );



        } catch (Exception e) { 
            e.printStackTrace();
            DriverStation.reportError("WHY IT NO WORKS AOPSDJKL:ASJDKL:ASJd", e.getStackTrace());
        }

    }
}
