package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.PhotonVision;

public class AprilTagTracking extends Command {
    private PhotonVision m_photonVision;
    Command m_command;

    public AprilTagTracking(PhotonVision photonVision, Command command) {
        m_photonVision = photonVision;
        m_command = command;
        addRequirements(photonVision);
    }

    @Override
    public void execute() {
        if (m_photonVision.seeingAprilTag()) {
            System.out.println("Seeing April Tag");
            // m_command.schedule();

            // try {
            //     PathPlannerPath path = PathPlannerPath.fromPathFile("complex1");
            //     AutoBuilder.followPath(path);
            // } catch (Exception e) {
            //     System.out.println("EXCEPTION!!! " + e);
            // }
        }
    }
}
