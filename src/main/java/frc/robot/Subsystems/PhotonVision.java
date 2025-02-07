package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    private PhotonCamera photonCamera;

    public PhotonVision() {
        photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
    }

    public boolean seeingAprilTag() {
        return photonCamera.getLatestResult().hasTargets();
    }

    public boolean seeingID22() {
        return photonCamera.getLatestResult().hasTargets() &&
        photonCamera.getLatestResult().getBestTarget().getFiducialId() == 22;
    }

    public boolean seeingID21() {
        return photonCamera.getLatestResult().hasTargets() &&
        photonCamera.getLatestResult().getBestTarget().getFiducialId() == 21;
    }
}
