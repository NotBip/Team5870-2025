package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignmentCommands.AutoAlignToReef;
import frc.robot.commands.SubsystemAlignmentCommands.Level4Align;

public class Testauto extends SequentialCommandGroup{
    
         public Testauto(SwerveSubsystem swerveSubsystem, Arm arm, Elevator elevator){

            new ParallelDeadlineGroup(new AutoAlignToReef(swerveSubsystem, 6, isScheduled()), new Level4Align(arm, elevator));
            

         }

}
