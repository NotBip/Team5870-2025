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
import frc.robot.commands.SubsystemAlignmentCommands.RestAlign;

public class SideRightRedTwoCoral extends SequentialCommandGroup {

    public SideRightRedTwoCoral(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator) {

        //does stuff
        addCommands(
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 11, true), new Level4Align(arm, elevator)), 
            new WaitCommand(1), 
            // would 
            new ParallelDeadlineGroup(new MoveLeft(swerveSubsystem), new RestAlign(arm, elevator)),  
            new RotateLeftTillAprilTag(swerveSubsystem, 1),
            new AutoAlignToSource(swerveSubsystem, 1),
            new WaitCommand(2),
            new ParallelCommandGroup(new MoveBack(swerveSubsystem, .5),new GrabCoral(elevator,arm)),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 6, true), new Level4Align(arm, elevator)),
            new WaitCommand(.5), 
            new MoveBack(swerveSubsystem, .3)            // ^ would take the rest of the time
            //give or take a few seconds we should still have time to do 2 coral auto, i dont think 3 is possible
        );
    }
}
