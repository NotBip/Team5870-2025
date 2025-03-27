package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.SubsystemAlignmentCommands.Level1Align;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;
import frc.robot.commands.SubsystemAlignmentCommands.RestAlign;

public class RedOneCoral extends SequentialCommandGroup {

    public RedOneCoral(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) {

        addCommands(
            new GrabCoral(elevator, arm),     
            new WaitCommand(.5),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 6, true), new Level4Align(arm, elevator)),
            new WaitCommand(2),
            new MoveBack(swerveSubsystem,.15),
            new RestAlign(arm, elevator).withTimeout(2)
        );
    }
}
