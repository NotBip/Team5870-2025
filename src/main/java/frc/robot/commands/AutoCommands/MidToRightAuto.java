package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToSource;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class MidToRightAuto extends SequentialCommandGroup {

    PathConstraints defaultContraints = new PathConstraints(4.1, 6, Units.degreesToRadians(540), Units.degreesToRadians(720));  


    public MidToRightAuto(SwerveSubsystem swerveSubsystem, Intake intake, Arm arm, Elevator elevator, boolean isRedAlliance) { 

        Pose2d initialPose; 
        int id1; 
        int id2; 
        double x; 
        double y; 
        double rot; 

        if (isRedAlliance) { 
            initialPose = new Pose2d(new Translation2d(11.675, 3.850), Rotation2d.fromDegrees(180));
            id1 = 10;
            id2 = 6; 
            x = 3.000; 
            y = 5.750; 
            rot = -55; 
        } else { 
            initialPose = new Pose2d(new Translation2d(5.875, 4.150), Rotation2d.fromDegrees(180)); 
            id1 = 21; 
            id2 = 19;  
            x = 14; 
            y = 2.5; 
            rot = -55; 
        }
        
        // new GrabCoral(elevator, arm); 
        new ParallelDeadlineGroup(
            new AutoAlignToReef(swerveSubsystem, id1, true), 
            new Level4Align(arm, elevator)
        );
        swerveSubsystem.findPathToPose(x, y, rot, defaultContraints, isRedAlliance);
        new AutoAlignToSource(swerveSubsystem, isRedAlliance, true); 
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
