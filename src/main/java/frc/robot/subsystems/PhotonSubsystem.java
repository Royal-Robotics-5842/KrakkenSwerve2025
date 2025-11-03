// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class PhotonSubsystem extends SubsystemBase {
  public boolean foundATarget = false;
  public Pose3d visionEstimatedRobotPose = null;
  private final PhotonCamera frontPhotonCameras = new PhotonCamera(Constants.PhotonConstants.frontCameras1);

  /** Creates a new PhotonSubsystem. */
  public PhotonSubsystem() {
  }

  @Override
  public void periodic() {

    foundATarget = frontPhotonCameras.getLatestResult().targets.size() > 0;
    frontPhotonCameras.getAllUnreadResults().forEach((result) -> {
      result.targets.forEach((target) -> {
        if(target.poseAmbiguity <= Constants.PhotonConstants.maxAllowedAmbiguity) {
          visionEstimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(target.getFiducialId())
              .get(),
          new Transform3d(0, 0, 0, new Rotation3d()));

      SmartDashboard.putNumberArray("Photon Robot Pose",
          new Double[] { (double) Math.round(visionEstimatedRobotPose.getX()), (double) Math.round(visionEstimatedRobotPose.getY()),
            (double) Math.round(visionEstimatedRobotPose.getZ())});
        }
      });
    });
    if(visionEstimatedRobotPose != null) {
      Constants.DataLoggingConstants.visionRelativeField.setRobotPose(visionEstimatedRobotPose.toPose2d());
    }
    SmartDashboard.putBoolean("Found a target", foundATarget);
  }
}
