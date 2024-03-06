package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro.Gyro;

public class SwerveDrive extends SubsystemBase {
  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  Gyro gyro;

  SwerveDriveOdometry odometry;

  public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, Gyro gyro) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.gyro = gyro;

    this.odometry = new SwerveDriveOdometry(Constants.SwerveDrive.PhysicalModel.kDriveKinematics, gyro.getRotation2d(), new SwerveModulePosition[]{
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(speeds);
    frontLeft.setState(moduleStates[0]);
    frontRight.setState(moduleStates[1]);
    backLeft.setState(moduleStates[2]);
    backRight.setState(moduleStates[3]);
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, gyro.getRotation2d()));
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
    drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), new SwerveModulePosition[]{
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });
  }
}
