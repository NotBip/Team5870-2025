package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToSource;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class SideBlueTwoCoral extends SequentialCommandGroup {

    public SideBlueTwoCoral(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) {

        //does stuff
        addCommands(
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 20, true), new Level4Align(arm, elevator)), 
            // ^ takes ~5 seconds with ~10 seconds left 
            new MoveRightSide(swerveSubsystem),
            new AutoAlignToSource(swerveSubsystem, 13),
            new WaitCommand(2),
            new ParallelCommandGroup(new MoveBack(swerveSubsystem, 1),new GrabCoral(elevator,arm)),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 19, true), new Level4Align(arm, elevator))
            // ^ would take the rest of the time

            //give or take a few seconds we should still have time to do 2 coral auto, i dont think 3 is possible
        );
    }
}
