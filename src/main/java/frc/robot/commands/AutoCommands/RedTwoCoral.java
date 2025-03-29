package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToSource;
import frc.robot.commands.SubsystemAlignmentCommands.Level1Align;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;
import frc.robot.commands.SubsystemAlignmentCommands.RestAlign;

public class RedTwoCoral extends SequentialCommandGroup {

    public RedTwoCoral(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) {

        //does stuff
        addCommands(
            // new GrabCoral(elevator, arm),     
            // new WaitCommand(.5),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 6, true), new Level1Align(arm, elevator)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(.5)), new MoveBack(swerveSubsystem, .4)),
            new MoveLeft(swerveSubsystem),
         new GetToCoralStation(swerveSubsystem, 12),
            new AutoAlignToSource(swerveSubsystem,true),
            new WaitCommand(1),
            new ParallelDeadlineGroup( new MoveBackReverse(swerveSubsystem, -.2)),
            new WaitCommand(.5),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 6, true), new Level1Align(arm, elevator)),
            new MoveBack(swerveSubsystem, .5)
            );



            
    }
}
