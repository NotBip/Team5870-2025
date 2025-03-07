package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class One_CoralAuto extends SequentialCommandGroup {

    public One_CoralAuto(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator, boolean isRedAlliance) {
        int id;
        if(isRedAlliance) { 
            id = 10; 
        }  else { 
            id = 21; 
        }

        addCommands(
            new GrabCoral(elevator, arm),     
            new WaitCommand(1.5),
            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, id, true), new Level4Align(arm, elevator))
        );
    }
}
