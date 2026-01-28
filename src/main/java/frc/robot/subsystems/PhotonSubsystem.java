// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;

public class PhotonSubsystem extends SubsystemBase {
  public boolean foundATarget = false;
  public Pose3d visionEstimatedRobotPose = null;
  private final PhotonCamera frontPhotonCameras = new PhotonCamera(Constants.PhotonConstants.frontCameras1);
  public final SwerveDrivePoseEstimator m_poseEstimator;
  public final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, new Transform3d(0,0,0, new Rotation3d()));
  private final SwerveSubsystem swerve;
  

  /** Creates a new PhotonSubsystem. */
  public PhotonSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.m_poseEstimator =  new SwerveDrivePoseEstimator(
      DrivetrainConstants.SwerveConstants.driveKinematics,
      swerve.getRotation2d(),
      swerve.getModulePositions(),
      Pose2d.kZero,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    foundATarget = frontPhotonCameras.getLatestResult().targets.size() > 0;
    frontPhotonCameras.getAllUnreadResults().forEach((result) -> {
      result.targets.forEach((target) -> {
        if (target.poseAmbiguity <= Constants.PhotonConstants.maxAllowedAmbiguity) {
          visionEstimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
              AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(target.getFiducialId())
                  .get(),
              new Transform3d(0, 0, 0, new Rotation3d())); //Position of camera on robot in meters

              photonPoseEstimator.setReferencePose(photonPoseEstimator.getReferencePose());
              Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(result);
              if(!pose.isEmpty()) {
                m_poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
              }
              
          SmartDashboard.putNumberArray("Photon Robot Pose",
              new Double[] { (double) Math.round(visionEstimatedRobotPose.getX()),
                  (double) Math.round(visionEstimatedRobotPose.getY()),
                  (double) Math.round(visionEstimatedRobotPose.getZ()) });
        }
      });
    });
    Constants.DataLoggingConstants.visionRelativeField.setRobotPose(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putBoolean("Found a target", foundATarget);

  }
}
