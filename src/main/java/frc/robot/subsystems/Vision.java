package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  PhotonCamera camera;

  // Target data
  double targetYaw, targetPitch;
  int targetID;
  boolean isTargetVisible = false;

  // Assume welded field layout for now, will change if we end up using the non-welded field
  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public Vision(String cameraName, boolean isDriverCamera) {
    System.out.println("[Vision] Initializing " + cameraName + " camera...");

    // Initialize the PhotonCamera with the given name and set it to driver mode if specified
    camera = new PhotonCamera(cameraName);

    // Set the camera to driver mode if specified (disables vision processing and reduces latency)
    camera.setDriverMode(isDriverCamera);

    System.out.println("[Vision] Camera " + cameraName + " initialized successfully!");
  }

  @Override
  public void periodic() {
    getAprilTagData();
  }

  /**
   * Get the latest camera result from PhotonVision.
   *
   * @return A list of all unread results from the camera, with the most recent result at the end of
   *     the list.
   */
  public List<PhotonPipelineResult> getLastCameraResult() {
    return camera.getAllUnreadResults();
  }

  /** Get the AprilTag data from the camera */
  public void getAprilTagData() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Get the most recent result
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          targetYaw = target.getYaw();
          targetPitch = target.getPitch();
          targetID = target.getFiducialId();
          isTargetVisible = true;
        }
      }
    } else {
      isTargetVisible = false;
      targetID = -1;
      targetPitch = 0;
      targetYaw = 0;
    }

    // Log the target data to AdvantageKit/AdvantageScope
    Logger.recordOutput("Vision/" + camera.getName() + "/Target ID", targetID);
    Logger.recordOutput("Vision/" + camera.getName() + "/Target Yaw", targetYaw);
    Logger.recordOutput("Vision/" + camera.getName() + "/Target Pitch", targetPitch);
    Logger.recordOutput("Vision/" + camera.getName() + "/Is Target Visible", isTargetVisible);
    Logger.recordOutput(
        "Vision/" + camera.getName() + "/Is Valid Tower Target", isValidTowerTarget());
    Logger.recordOutput("Vision/" + camera.getName() + "/Is Valid Hub Target", isValidHubTarget());
    Logger.recordOutput("Vision/" + camera.getName() + "/All Unread Results", results.toString());
  }

  /** Return the yaw of the target */
  public double getTargetYaw() {
    return targetYaw;
  }

  /** Return the pitch of the target */
  public double getTargetPitch() {
    return targetPitch;
  }

  /** Return the ID of the target */
  public int getTargetID() {
    return targetID;
  }

  /** Return whether the target is visible */
  public boolean isTargetVisible() {
    return isTargetVisible;
  }

  /**
   * Return whether the target is a valid tower target based on its ID and alliance
   *
   * @return true if the target is a valid tower target, false otherwise
   */
  public boolean isValidTowerTarget() {
    if (!isTargetVisible || targetID == -1) {
      return false;
    }
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        // Red alliance targets
        if (targetID == 15 || targetID == 16) {
          return true;
        }
      } else {
        // Blue alliance targets
        if (targetID == 31 || targetID == 32) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Return whether the target is a valid hub target based on its ID and alliance
   *
   * @return true if the target is a valid hub target, false otherwise
   */
  public boolean isValidHubTarget() {
    if (!isTargetVisible || targetID == -1) {
      return false;
    }
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        // Red alliance targets
        if (targetID == 9 || targetID == 10) {
          return true;
        }
      } else {
        // Blue alliance targets
        if (targetID == 25 || targetID == 26) {
          return true;
        }
      }
    }
    return false;
  }
}
