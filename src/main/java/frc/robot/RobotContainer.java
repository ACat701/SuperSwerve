package frc.robot;

import java.util.List;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.DriveSimple;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final Drivetrain subDrivetrain = new Drivetrain();

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new DriveSimple(subDrivetrain, conDriver, true, true));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    conDriver.btn_X.whenPressed(new InstantCommand(() -> subDrivetrain.zeroGyroYaw()));
    conDriver.btn_Y.whenPressed(new InstantCommand(() -> subDrivetrain.configure()));
  }

  public Command getAutonomousCommand() {

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d()),
        List.of(new Translation2d(1, 0), new Translation2d(1, 1), new Translation2d(0, 1)),
        new Pose2d(0, 0, new Rotation2d()),
        new TrajectoryConfig(Units.feetToMeters(3), Units.feetToMeters(3)));

    return null;
  }
}
