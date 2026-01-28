// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;

public class SwerveSubsystem extends SubsystemBase {
  public SwerveModule backLeft = new SwerveModule(
      DrivetrainConstants.SwerveConstants.backLeftTurn,
      DrivetrainConstants.SwerveConstants.backLeftDrive,
      DrivetrainConstants.SwerveConstants.backLeftEncoder);

  public SwerveModule backRight = new SwerveModule(
      DrivetrainConstants.SwerveConstants.backRightTurn,
      DrivetrainConstants.SwerveConstants.backRightDrive,
      DrivetrainConstants.SwerveConstants.backRightEncoder);

  public SwerveModule frontLeft = new SwerveModule(
      DrivetrainConstants.SwerveConstants.frontLeftTurn,
      DrivetrainConstants.SwerveConstants.frontLeftDrive,
      DrivetrainConstants.SwerveConstants.frontLeftEncoder);

  public SwerveModule frontRight = new SwerveModule(
      DrivetrainConstants.SwerveConstants.frontRightTurn,
      DrivetrainConstants.SwerveConstants.frontRightDrive,
      DrivetrainConstants.SwerveConstants.frontRightEncoder);

  SendableChooser<Double> speed_chooser = new SendableChooser<>();
  SendableChooser<Boolean> field_oriented_Chooser = new SendableChooser<>();
  RobotConfig config;

  public ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);;
  public SwerveDriveOdometry swerveDriveOdometry;
  public Pigeon2 gyro;

  public SwerveSubsystem(int gyroPort) {
    this.gyro = new Pigeon2(gyroPort);

    swerveDriveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SwerveConstants.driveKinematics, getRotation2d(),
        getModulePositions());

    speed_chooser.addOption("Fast", DrivetrainConstants.ChasisConstants.fast);
    speed_chooser.addOption("Slow", DrivetrainConstants.ChasisConstants.slow);
    speed_chooser.addOption("Slowest", DrivetrainConstants.ChasisConstants.slowest);
    speed_chooser.addOption("Precision", DrivetrainConstants.ChasisConstants.precision);
    speed_chooser.setDefaultOption("Normal", DrivetrainConstants.ChasisConstants.normal);
    SmartDashboard.putData("Swerve Speed", speed_chooser);

    field_oriented_Chooser.addOption("Robot", false);
    field_oriented_Chooser.setDefaultOption("Field", true);
    SmartDashboard.putData("Robot Oritentation", field_oriented_Chooser);

    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(0.01, 0.0, 0.0),
              new PIDConstants(4.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
    ;
  }

  public Command followTrajectory(int holonomicEndGoalState, Pose2d... poses) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(
        DrivetrainConstants.SwerveConstants.kPhysicalMaxSpeedMetersPerSecond,
        DrivetrainConstants.SwerveConstants.maxAngularAccelerationUnitsPerSecond, 2 * Math.PI, 4 * Math.PI);
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // On the fly path, no ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(holonomicEndGoalState)));

    path.preventFlipping = true;
    Command trajectory = AutoBuilder.followPath(path);
    return trajectory;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelative) {
    SwerveModuleState[] targetStates = DrivetrainConstants.SwerveConstants.driveKinematics
        .toSwerveModuleStates(robotRelative);
    setModuleStates(targetStates);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getDriveInMeter(),
        frontRight.getDriveInMeter(),
        backLeft.getDriveInMeter(),
        backRight.getDriveInMeter()
    };
  }

  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    swerveDriveOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[] {
        frontRight.getState(),
        frontLeft.getState(),
        backRight.getState(),
        backLeft.getState()
    };
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return chassisSpeeds;
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    swerveDriveOdometry.update(getRotation2d(), getModulePositions());
    Constants.DataLoggingConstants.odometryRelativeField
        .setRobotPose(RobotContainer.swerveSubsystem.swerveDriveOdometry.getPoseMeters());
    SmartDashboard.putData("Odometry Field", Constants.DataLoggingConstants.odometryRelativeField);
    SmartDashboard.putData("Vision Field", Constants.DataLoggingConstants.visionRelativeField);
    Pose2d pose = swerveDriveOdometry.getPoseMeters();
    double distanceTraveled = Math.sqrt(Math.pow(pose.getX() - 0, 2) + Math.pow(pose.getY() - 0, 2));

    SmartDashboard.putNumber("Rotation", getHeading());
    SmartDashboard.putNumber("FL", Math.abs(frontLeft.getTurningPosition()));
    SmartDashboard.putNumber("FR", Math.abs(frontRight.getTurningPosition()));
    SmartDashboard.putNumber("BL", Math.abs(backLeft.getTurningPosition()));
    SmartDashboard.putNumber("BR", Math.abs(backRight.getTurningPosition()));
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);

    DrivetrainConstants.SwerveConstants.fieldOriented = field_oriented_Chooser.getSelected().booleanValue();
    DrivetrainConstants.ChasisConstants.speedLimiter = (Double) speed_chooser.getSelected();
  }
}
