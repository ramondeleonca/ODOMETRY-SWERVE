package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.constants.SwerveModuleOptions;
import lib.team3526.control.LazyCANSparkMax;
import lib.team3526.control.LazySparkPID;

public class SwerveModule extends SubsystemBase {
  private SwerveModuleOptions options;

  private final LazyCANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;

  private final LazyCANSparkMax turningMotor;
  private final RelativeEncoder turningEncoder;
  private final LazySparkPID turningPID;

  private final CANcoder absoluteEncoder;

  private SwerveModuleState targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

  public SwerveModule(SwerveModuleOptions options) {
    // Store options
    this.options = options;

    //! DRIVE MOTOR
    // Configure drive motor
    this.driveMotor = new LazyCANSparkMax(options.driveMotorID, LazyCANSparkMax.MotorType.kBrushless);
    this.driveMotor.setInverted(options.driveMotorInverted);

    // Configure drive encoder
    this.driveEncoder = driveMotor.getEncoder();
    this.driveEncoder.setPositionConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RotationToMeter); 
    this.driveEncoder.setVelocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond);

    //! TURNING MOTOR
    // Configure turning motor
    this.turningMotor = new LazyCANSparkMax(options.turningMotorID, LazyCANSparkMax.MotorType.kBrushless);
    this.turningMotor.setInverted(options.turningMotorInverted);
    this.turningPID = new LazySparkPID(turningMotor);
    PIDFConstants.applyToSparkPIDController(this.turningPID.controller, Constants.SwerveDrive.SwerveModules.kTurningPIDConstants);
    this.turningPID.controller.setPositionPIDWrappingMinInput(0);
    this.turningPID.controller.setPositionPIDWrappingMaxInput(2 * Math.PI);
    this.turningPID.controller.setPositionPIDWrappingEnabled(true);

    // Configure turning encoder
    this.turningEncoder = turningMotor.getEncoder();
    this.turningEncoder.setPositionConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RotationToRadian); 
    this.turningEncoder.setVelocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RPMToRadianPerSecond);

    //! ABSOLUTE ENCODER
    this.absoluteEncoder = new CANcoder(options.absoluteEncoderDevice.getDeviceID(), options.absoluteEncoderDevice.getCanbus());

    // Initialize encoder (reset)
    resetEncoders();
  }

  /**
   * Gets the real, absolute position from the CANCoder
   * @return
   */
  public Measure<Angle> getAbsoluteAngle() {
    return Rotations.of(absoluteEncoder.getAbsolutePosition().refresh().getValue()).times(this.options.absoluteEncoderInverted ? -1.0 : 1.0);
  }

  /**
   * Gets expected position from the turning encoder
   */
  public Measure<Angle> getTurningAngle() {
    return Radians.of(turningEncoder.getPosition() % (2 * Math.PI));
  }

  /**
   * Resets the drive encoder (zero)
   */
  public void resetDriveEncoder() {
    this.driveEncoder.setPosition(0);
  }

  /**
   * Resets the turn encoder (real wheel rotation)
   */
  public void resetTurningEncoder() {
    this.turningEncoder.setPosition(getAbsoluteAngle().in(Radians));
  }

  /**
   * Resets both turn and drive encoders
   */
  public void resetEncoders() {
    resetDriveEncoder();
    resetTurningEncoder();
  }

  /**
   * Sets the state, optionally forcefully
   * @param state
   */
  public void setState(SwerveModuleState state) {
    setState(state, false);
  }

  /**
   * Stops the module
   */
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  /**
   * Sets the rotation and speed state of the module
   * @param state
   * @param force
   */
  public void setState(SwerveModuleState state, boolean force) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001 || force) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, new Rotation2d(getTurningAngle()));

    this.targetState = state;

    driveMotor.set(state.speedMetersPerSecond / Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
    turningPID.setReference(state.angle.getRadians(), ControlType.kPosition);
  }

  /**
   * Gets the position of the module relative to the field
   * @return
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.driveEncoder.getPosition(), Rotation2d.fromRadians(this.getAbsoluteAngle().in(Radians)));
  }

  /**
   * Gets the state of the module
   * @return
   */
  public SwerveModuleState getTargetState() {
    return this.targetState;
  }

  /**
   * Gets the state of the module
   * @return
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.driveEncoder.getVelocity(), Rotation2d.fromRadians(this.getAbsoluteAngle().in(Radians)));
  }

  @Override
  public void periodic() {
    
  }
}
