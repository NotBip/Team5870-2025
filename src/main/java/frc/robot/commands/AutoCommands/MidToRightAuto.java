package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class MidToRightAuto extends SequentialCommandGroup {

    PathConstraints defaultContraints = new PathConstraints(4.1, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));  
    private Pose2d pose1; 
    private Pose2d pose2; 
    private Pose2d pose3; 

    public PathPlannerPath getAutoFromPathFile(String pathName) {
        try{
            return PathPlannerPath.fromPathFile("pathName");
        }
        catch(Exception e){
            e.printStackTrace();
        }
        return null;
    }

    public MidToRightAuto(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) { 
        try { 
            addCommands(
                new GrabCoral(elevator, arm),     
                new WaitCommand(1.5),
                new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 21, true), new Level4Align(arm, elevator)),
                new InstantCommand(() -> swerveSubsystem.resetOdometry(getAutoFromPathFile("MidToRightAuto1").getStartingHolonomicPose().get())), 
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("MidToRightAuto1")),
                new AutoAlignToSource(swerveSubsystem, false, true)

            );    
        } catch (Exception e) { 
            e.printStackTrace();
            Commands.none();
        }
        
    }
    
}
