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

public class BlueTwoCoral extends SequentialCommandGroup {

    public BlueTwoCoral(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) {

        //does stuff
        addCommands(
            // new GrabCoral(elevator, arm),     
            // new WaitCommand(.5),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 21, true), new Level4Align(arm, elevator)), 
            // ^ takes ~5 seconds with ~10 seconds left 
            new MoveRight(swerveSubsystem),
            new GetToCoralStation(swerveSubsystem, 13),
            new AutoAlignToSource(swerveSubsystem, 13),
            // ^ takes ~3 seconds with ~7 seconds left
            new WaitCommand(2),
            new ParallelCommandGroup(new MoveBack(swerveSubsystem),new GrabCoral(elevator,arm)),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 19, true), new Level4Align(arm, elevator))
            // ^ would take the rest of the time

            //give or take a few seconds we should still have time to do 2 coral auto, i dont think 3 is possible
        );
    }
}
