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
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class RedTwoCoral extends SequentialCommandGroup {

    public RedTwoCoral(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) {

        //does stuff
        addCommands(
            // new GrabCoral(elevator, arm),     
            // new WaitCommand(.5),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 10, true), new Level4Align(arm, elevator)), 
            new MoveRight(swerveSubsystem), 
            new GetToCoralStation(swerveSubsystem, 1),
            new AutoAlignToSource(swerveSubsystem,1),
            new WaitCommand(2),
            new ParallelCommandGroup(new MoveBack(swerveSubsystem, 1),new GrabCoral(elevator,arm)),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 6, true), new Level4Align(arm, elevator))
            
        );
    }
}
