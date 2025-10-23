// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveSubsystem extends SubsystemBase {
  SendableChooser<Double> speed_chooser = new SendableChooser<>();
    public SwerveModule backLeft = new SwerveModule(
      Constants.ServeConstants.backLeftTurn,
      Constants.ServeConstants.backLeftDrive,
      Constants.ServeConstants.backLeftEncoder, false, false
     );

     public SwerveModule backRight = new SwerveModule(
      Constants.ServeConstants.backRightTurn,
      Constants.ServeConstants.backRightDrive,
      Constants.ServeConstants.backRightEncoder, false, false
     );

     public SwerveModule frontLeft = new SwerveModule(
      Constants.ServeConstants.frontLeftTurn,
      Constants.ServeConstants.frontLeftDrive,
      Constants.ServeConstants.frontLeftEncoder, false, false
     );

     public SwerveModule frontRight = new SwerveModule(
      Constants.ServeConstants.frontRightTurn,
      Constants.ServeConstants.frontRightDrive,
      Constants.ServeConstants.frontRightEncoder, false, false
     );

     RobotConfig config;
     ChassisSpeeds chassisSpeeds;
     public SwerveDriveOdometry swerveDriveOdometry;
    
     public Pigeon2 gyro;


  public SwerveSubsystem(int gyroPort) {
    this.gyro = new Pigeon2(gyroPort);
    swerveDriveOdometry = new SwerveDriveOdometry(Constants.ServeConstants.driveKinematics, getRotation2d(), getModulePositions());

    speed_chooser.addOption("Fast",Constants.ChasisConstants.fast);
    speed_chooser.addOption("Slow", Constants.ChasisConstants.slow);
    speed_chooser.addOption("Slowest", Constants.ChasisConstants.slowest);
    speed_chooser.addOption("Precision", Constants.ChasisConstants.precision);
    speed_chooser.setDefaultOption("Normal", Constants.ChasisConstants.normal);
    SmartDashboard.putData("Swerve Speed", speed_chooser);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelative) 
  {
    SwerveModuleState[] targetStates = Constants.ServeConstants.driveKinematics.toSwerveModuleStates(robotRelative);
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

  public void stopModules() 
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) 
  {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]); 
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates()
  {
    SwerveModuleState[] states = new SwerveModuleState[] {
      frontRight.getState(),
      frontLeft.getState(),
      backRight.getState(),
      backLeft.getState()
    };
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() 
  {
    return chassisSpeeds;
  }


  public void zeroHeading() 
  {
    gyro.reset();
  }
  
  public double getHeading() 
  {
    return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
  }
    
  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading()); //For FRC functions, just converts where your facing to a "Rotation2d" type
  }
  
  @Override
  public void periodic() {
    swerveDriveOdometry.update(getRotation2d(), getModulePositions());
    Constants.ShuffleboardConstants.glassField2d.setRobotPose(RobotContainer.swerveSubsystem.swerveDriveOdometry.getPoseMeters());
    SmartDashboard.putData("Field", Constants.ShuffleboardConstants.glassField2d);

    Pose2d pose = swerveDriveOdometry.getPoseMeters();
    double distanceTraveled = Math.sqrt(Math.pow(pose.getX() - 0, 2) + Math.pow(pose.getY() - 0, 2));


    SmartDashboard.putNumber("Rotation", getHeading());
    SmartDashboard.putNumber("FL", Math.abs(frontLeft.getTurningPosition()));
    SmartDashboard.putNumber("FR", Math.abs(frontRight.getTurningPosition()));
    SmartDashboard.putNumber("BL", Math.abs(backLeft.getTurningPosition()));
    SmartDashboard.putNumber("BR", Math.abs(backRight.getTurningPosition()));
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);

    Constants.ChasisConstants.speedLimiter = (Double) speed_chooser.getSelected();
  }
}
