package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPreferences;
import frc.robot.SN_SwerveModule;
import frc.robot.Constants.Module0;
import frc.robot.Constants.Module1;
import frc.robot.Constants.Module2;
import frc.robot.Constants.Module3;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;

public class Drivetrain extends SubsystemBase {

  private SN_SwerveModule[] swerveModules;
  private PigeonIMU pigeon;
  private SwerveDriveOdometry odometry;

  public SlewRateLimiter driveXSlewRateLimiter;
  public SlewRateLimiter driveYSlewRateLimiter;
  public SlewRateLimiter steerSlewRateLimiter;

  public Drivetrain() {

    swerveModules = new SN_SwerveModule[] {
        new SN_SwerveModule(Module0.CONSTANTS),
        new SN_SwerveModule(Module1.CONSTANTS),
        new SN_SwerveModule(Module2.CONSTANTS),
        new SN_SwerveModule(Module3.CONSTANTS)
    };

    pigeon = new PigeonIMU(mapDrivetrain.PIGEON_CAN);
    zeroGyroYaw();

    odometry = new SwerveDriveOdometry(Constants.SWERVE_KINEMATICS, getGyroYaw());

    driveXSlewRateLimiter = new SlewRateLimiter(prefDrivetrain.driveRateLimit.getValue());
    driveYSlewRateLimiter = new SlewRateLimiter(prefDrivetrain.driveRateLimit.getValue());
    steerSlewRateLimiter = new SlewRateLimiter(prefDrivetrain.steerRateLimit.getValue());

    configure();
  }

  /**
   * Configure each swerve module and the Pigeon IMU
   */
  public void configure() {
    for (SN_SwerveModule mod : swerveModules) {
      mod.configure();
    }
    pigeon.configFactoryDefault();
  }

  /**
   * Drive the drivetrain
   * 
   * @param velocity      Desired translational and rotational velocity in meters
   *                      and radians per second respectively
   * @param fieldRelative Is the desired translational velocity field relative or
   *                      robot relative
   * @param isOpenLoop    Is the drive motor velocity controlled using
   *                      open or closed loop control
   */
  public void drive(Pose2d velocity, boolean fieldRelative, boolean isOpenLoop) {

    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          velocity.getX(),
          velocity.getY(),
          velocity.getRotation().getRadians(),
          getGyroYaw());
    } else {
      chassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), velocity.getRotation().getRadians());
    }
    SwerveModuleState[] states = Constants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    // mutates states with desaturated wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Units.feetToMeters(prefDrivetrain.maxSpeedFPS.getValue()));

    for (SN_SwerveModule mod : swerveModules) {
      mod.setDesiredState(states[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Directly set the state of each swerve module. Uses closed loop control for
   * drive velocity
   * 
   * @param desiredStates List of each desired state
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        Units.feetToMeters(prefDrivetrain.maxSpeedFPS.getValue()));

    for (SN_SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * Get the state (velocity and angle) of each swerve module
   * 
   * @return State of each swerve module
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState states[] = new SwerveModuleState[swerveModules.length];

    for (SN_SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  /**
   * Get the yaw as reported by the Pigeon IMU
   * 
   * @return Yaw of drivetrain
   */
  public Rotation2d getGyroYaw() {
    double yawDegress = pigeon.getYaw();
    yawDegress *= constDrivetrain.GYRO_INVERT ? -1 : 1;
    return Rotation2d.fromDegrees(yawDegress);
  }

  public void zeroGyroYaw() {
    pigeon.setYaw(0);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(getGyroYaw(), getModuleStates());

    if (RobotPreferences.displayPreferences.getValue()) {

      for (SN_SwerveModule mod : swerveModules) {

        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Steer Encoder",
            mod.getSteerEncoder().getDegrees());
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Drive Motor Velocity ",
            Units.metersToFeet(mod.getState().speedMetersPerSecond));
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Steer Motor Angle ",
            mod.getState().angle.getDegrees());

      }

      SmartDashboard.putNumber("Drivetrain Gyro Yaw", getGyroYaw().getDegrees());

      SmartDashboard.putNumber("Drivetrain Pose X", getPose().getX());
      SmartDashboard.putNumber("Drivetrain Pose Y", getPose().getY());
      SmartDashboard.putNumber("Drivetrain Pose Rotation", getPose().getRotation().getDegrees());

    }
  }
}
