// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShuffleboardConstants {
    public static Field2d glassField2d = new Field2d();
  }

  public static class ChasisConstants {
    public static final int pidgeonGyro = 0;
    public static Double speedLimiter = 1.0;

    public static Double normal = 1.0;
    public static Double fast = 1.5;
    public static Double slow = 4.0;
    public static Double slowest = 6.0;
    public static Double precision = 10.0;
  }

  public static class ServeConstants {
    public static final double trackwidth = edu.wpi.first.math.util.Units.inchesToMeters(28.5); // Distance between right and left wheels
    public static final double wheelBase = edu.wpi.first.math.util.Units.inchesToMeters(28.5);  // Distance between front and back wheels

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackwidth / 2), //FL
            new Translation2d(wheelBase / 2, -trackwidth / 2), //FR
            new Translation2d(-wheelBase / 2, trackwidth / 2), //BL
            new Translation2d(-wheelBase / 2, -trackwidth / 2)); //BR
    
    //Front Left Swerve
    public static final int frontLeftTurn = 3;
    public static final int frontLeftDrive = 11;

    //Front Right Swerve
    public static final int frontRightTurn = 10;
    public static final int frontRightDrive = 7;

    //Back Left Swerve
    public static final int backLeftTurn = 2;
    public static final int backLeftDrive = 1;

    //Back Right Swerve
    public static final int backRightTurn = 12;
    public static final int backRightDrive = 13;

    //Encoders
    public static final int backRightEncoder = 21;
    public static final int backLeftEncoder = 5;
    public static final int frontRightEncoder = 6;
    public static final int frontLeftEncoder = 4;

    //Math Stuff
    public static final int defaultStatorLimit = 20;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.9436; //https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double maxAccelerationUnitsPerSecond = 3.5;
    public static final double maxAngularAccelerationUnitsPerSecond = 3.5;
  }

  public static final class OIConstants
  {
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 5;


    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
    public static final double kDeadband = 0.06;
  }
}


