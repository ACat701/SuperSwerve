// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToPosition extends CommandBase {

  Drivetrain subDrivetrain;

  public DriveToPosition(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = subDrivetrain.driveController.calculate(
        subDrivetrain.getPose(),
        new Pose2d(0, 0, new Rotation2d(0)),
        1,
        new Rotation2d());

    subDrivetrain.setModuleStates(Constants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
