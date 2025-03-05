package frc.robot.commands.AutoCommands;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToSource;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class TestAuto extends SequentialCommandGroup {

    PathConstraints defaultContraints = new PathConstraints(4.1, 6, Units.degreesToRadians(540), Units.degreesToRadians(720));  
    private Pose2d initialPose = new Pose2d(); 

    public TestAuto(SwerveSubsystem swerveSubsystem, Intake intake, Arm arm, Elevator elevator, boolean isRedAlliance) { 
        int id1; //javiera did everything
        int id2; 
        double x; 
        double y; 
        double rot; 

        if (isRedAlliance) { 
            try {
                initialPose = PathPlannerPath.fromPathFile("TestPath").flipPath().getStartingHolonomicPose().get();
            } catch (Exception e) { 
                e.printStackTrace();
            }  
            id1 = 10;
            id2 = 6; 
            x = 3.000; 
            y = 5.750; 
            rot = -55; 
        } else { 
            try {
                initialPose = PathPlannerPath.fromPathFile("TestPath").getStartingHolonomicPose().get();
            } catch (Exception e) { 
                e.printStackTrace();
            }
            id1 = 21; 
            id2 = 19;  
            x = 14; 
            y = 2.5; 
            rot = -55; 
        }   

        new InstantCommand(() -> swerveSubsystem.resetOdometry(initialPose)); 
        try {
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestPath"));
        } catch (Exception e) { 
            e.printStackTrace();
        }
         new ParallelDeadlineGroup(
            new AutoAlignToReef(swerveSubsystem, id2, true), 
            new Level4Align(arm, elevator)
        );
        swerveSubsystem.findPathToPose(x, y, rot, defaultContraints, isRedAlliance);
        new AutoAlignToSource(swerveSubsystem, isRedAlliance, true);
        new WaitCommand(2);  
        new ParallelRaceGroup(
            swerveSubsystem.findPathToPose(x, y, -60, defaultContraints, isRedAlliance),
            new GrabCoral(elevator, arm)
        ); 
        new ParallelDeadlineGroup(
            new AutoAlignToReef(swerveSubsystem, id2, true), 
            new Level4Align(arm, elevator)
        );
        new AutoAlignToSource(swerveSubsystem, isRedAlliance, true); 
        new ParallelRaceGroup(
            swerveSubsystem.findPathToPose(x, y, -60, defaultContraints, isRedAlliance),
            new GrabCoral(elevator, arm)
        ); 
        new ParallelDeadlineGroup(
            new AutoAlignToReef(swerveSubsystem, id2, false), 
            new Level4Align(arm, elevator)
        );
    }
    
}
