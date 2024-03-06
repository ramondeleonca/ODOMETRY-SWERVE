// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);

  Gyro gyro;

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  SwerveDrive swerveDrive;

  public RobotContainer() {
    this.gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));
    
    this.frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
    this.frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
    this.backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
    this.backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

    this.swerveDrive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, gyro);

    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(new DriveSwerve(swerveDrive, 
      () -> controller.getLeftY(),
      () -> controller.getLeftX(),
      () -> controller.getRightX(),
      () -> controller.a().getAsBoolean()
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
