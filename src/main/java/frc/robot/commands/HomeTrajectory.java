// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeTrajectory extends Command {
  private final SwerveSubsystem swerve;
  private Command trajectoryCommand;
  /** Creates a new HomeTrajectory. */
  public HomeTrajectory(SwerveSubsystem swerveDrive) {
    this.swerve = swerveDrive;
    addRequirements(swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectoryCommand = swerve.followTrajectory(0, swerve.swerveDriveOdometry.getPoseMeters(), new Pose2d(0,0, new Rotation2d(0)));
    trajectoryCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("test");
    trajectoryCommand.end(interrupted);
  }

  @Override
  public void cancel() {
      // TODO Auto-generated method stub
      super.cancel();
      trajectoryCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

