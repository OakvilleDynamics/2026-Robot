package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  PhotonCamera climberCamera, shooterCamera, reverseCamera;

  // Array of all cameras for easy iteration
  PhotonCamera[] cameras = {climberCamera, shooterCamera, reverseCamera};

  // Target data
  double targetYaw, targetPitch;
  int targetID;

  boolean isTargetVisible;

  // Assume welded field layout for now, will change if we end up using the non-welded field
  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public Vision() {
    climberCamera = new PhotonCamera("Climber Camera");
    shooterCamera = new PhotonCamera("Shooter Camera");
    reverseCamera = new PhotonCamera("Reverse Camera");
  }

  /**
   * Gets the climber camera PhotonVision instance.
   *
   * @return Climber camera PhotonVision instance.
   */
  public PhotonCamera getClimberCamera() {
    return climberCamera;
  }

  /**
   * Gets the shooter camera PhotonVision instance.
   *
   * @return Shooter camera PhotonVision instance.
   */
  public PhotonCamera getShooterCamera() {
    return shooterCamera;
  }

  /**
   * Gets the reverse camera PhotonVision instance.
   *
   * @return Reverse camera PhotonVision instance.
   */
  public PhotonCamera getReverseCamera() {
    return reverseCamera;
  }

  public List<PhotonPipelineResult> getLastCameraResult(PhotonCamera camera) {
    return camera.getAllUnreadResults();
  }

  public void printCameraResults(PhotonCamera camera) {
    List<PhotonPipelineResult> results = getLastCameraResult(camera);
    for (PhotonPipelineResult result : results) {
      System.out.println("Camera: " + camera.getName());
      System.out.println("Has Targets: " + result.hasTargets());
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          System.out.println("Target ID: " + target.getFiducialId());
          System.out.println("Pose: " + target.getBestCameraToTarget().toString());
        }
      }
    }
  }

  public void printAllCameraResults() {
    for (PhotonCamera camera : cameras) {
      printCameraResults(camera);
    }
  }

  public void getAprilTagData(PhotonCamera camera) {
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
    }
    isTargetVisible = false;
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
}
