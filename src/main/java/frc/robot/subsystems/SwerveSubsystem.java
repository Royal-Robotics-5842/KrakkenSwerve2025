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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;

public class SwerveSubsystem extends SubsystemBase {
  SendableChooser<Double> speed_chooser = new SendableChooser<>();
    public SwerveModule backLeft = new SwerveModule(
      DrivetrainConstants.ServeConstants.backLeftTurn,
      DrivetrainConstants.ServeConstants.backLeftDrive,
      DrivetrainConstants.ServeConstants.backLeftEncoder, false, false
     );

     public SwerveModule backRight = new SwerveModule(
      DrivetrainConstants.ServeConstants.backRightTurn,
      DrivetrainConstants.ServeConstants.backRightDrive,
      DrivetrainConstants.ServeConstants.backRightEncoder, false, false
     );

     public SwerveModule frontLeft = new SwerveModule(
      DrivetrainConstants.ServeConstants.frontLeftTurn,
      DrivetrainConstants.ServeConstants.frontLeftDrive,
      DrivetrainConstants.ServeConstants.frontLeftEncoder, false, false
     );

     public SwerveModule frontRight = new SwerveModule(
      DrivetrainConstants.ServeConstants.frontRightTurn,
      DrivetrainConstants.ServeConstants.frontRightDrive,
      DrivetrainConstants.ServeConstants.frontRightEncoder, false, false
     );

     RobotConfig config;
     ChassisSpeeds chassisSpeeds;
     public SwerveDriveOdometry swerveDriveOdometry;
    
     public Pigeon2 gyro;


  public SwerveSubsystem(int gyroPort) {
    this.gyro = new Pigeon2(gyroPort);
    swerveDriveOdometry = new SwerveDriveOdometry(DrivetrainConstants.ServeConstants.driveKinematics, getRotation2d(), getModulePositions());

    speed_chooser.addOption("Fast",DrivetrainConstants.ChasisConstants.fast);
    speed_chooser.addOption("Slow", DrivetrainConstants.ChasisConstants.slow);
    speed_chooser.addOption("Slowest", DrivetrainConstants.ChasisConstants.slowest);
    speed_chooser.addOption("Precision", DrivetrainConstants.ChasisConstants.precision);
    speed_chooser.setDefaultOption("Normal", DrivetrainConstants.ChasisConstants.normal);
    SmartDashboard.putData("Swerve Speed", speed_chooser);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelative) 
  {
    SwerveModuleState[] targetStates = DrivetrainConstants.ServeConstants.driveKinematics.toSwerveModuleStates(robotRelative);
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

    DrivetrainConstants.ChasisConstants.speedLimiter = (Double) speed_chooser.getSelected();
  }
}
