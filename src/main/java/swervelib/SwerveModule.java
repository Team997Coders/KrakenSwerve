// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swervelib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
  private TalonFX angleMotor;
  private TalonFX speedMotor;
  private PIDController pidController;
  private Canandmag absoluteEncoder;
  private double maxVelocity;
  private double maxVoltage;
  private TalonFXConfiguration speedConfig;
  private TalonFXConfiguration turnConfig;

 private double driveReduction = 1.0 / 6.75;
 private double WHEEL_DIAMETER = 0.1016;
 private double rotationsToDistance = driveReduction * WHEEL_DIAMETER * Math.PI;

 private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

  public SwerveModule(int angleMotorId, int speedMotorId, int encoderId, boolean driveMotorReversed, boolean angleMotorReversed,
      boolean angleEncoderReversed, double angleEncoderConversionFactor, double angleEncoderOffset,
      double maxVelocity, double maxVoltage) {

        constants = new SwerveModuleConstants<>();
    this.angleMotor = new TalonFX(angleMotorId);
    this.speedMotor = new TalonFX(speedMotorId);

    speedConfig = new TalonFXConfiguration();
    speedConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    speedConfig.Slot0 = constants.DriveMotorGains;
    speedConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    speedConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    speedConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    speedConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    speedConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    speedMotor.getConfigurator().apply(speedConfig, 0.25);

    turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;

    angleMotor.getConfigurator().apply(speedConfig, 0.25);

    this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
    this.absoluteEncoder =  new Canandmag(encoderId);
    //this.absoluteEncoder.getSettings().
    this.maxVelocity = maxVelocity;
    this.maxVoltage = maxVoltage;

    this.pidController.enableContinuousInput(-180, 180);

    angleMotor.setPosition(absoluteEncoder.getAbsPosition()*360);


    // SparkBaseConfig angleMotorConfig = new SparkMaxConfig();
    //     angleMotorConfig
    //       .inverted(angleMotorReversed)
    //       .idleMode(IdleMode.kBrake);
    //     angleMotorConfig.absoluteEncoder
    //       .positionConversionFactor(1)
    //       .velocityConversionFactor(1)
    //       .inverted(angleEncoderReversed);

    // SparkBaseConfig speedMotorConfig = new SparkMaxConfig();
    //     speedMotorConfig
    //       .inverted(driveMotorReversed)
    //       .idleMode(IdleMode.kBrake);
    //     speedMotorConfig.encoder
    //       .positionConversionFactor(rotationsToDistance)
    //       .velocityConversionFactor(rotationsToDistance/60);

    
    
    //angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    //speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);

    //this.encoder.setZeroOffset(angleEncoderOffset);
  }

  public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
    this(config.angleMotorId,
        config.driveMotorId,
        config.encoderId,
        config.driveMotorReversed,
        config.angleMotorReversed,
        config.angleEncoderReversed,
        config.angleEncoderConversionFactor,
        config.angleEncoderOffset,
        maxVelocity,
        maxVoltage);
  }

  /**
   * drive:
   * 
   * @param speedMetersPerSecond
   * @param angle
   *                             Basic drive code. Lots of debug information sent
   *                             to the dashboard so that
   *                             we can watch what is happening. Remember that
   *                             everything should be CCW positive.
   */
  private void drive(double speedMetersPerSecond, double angle) {
    double drive_voltage = (speedMetersPerSecond / maxVelocity) * maxVoltage;
    double angle_voltage = pidController.calculate( angle);

    speedMotor.setVoltage(drive_voltage);
    angleMotor.setVoltage(angle_voltage);
  }

  /**
   * drive:
   * 
   * @param state of the module (velocity and angle)
   */
  public void drive(SwerveModuleState state) {
    state.optimize(getRotation());

    // a little wierd logic. Call the other 'drive' code above to actually move the
    // module.
    this.drive(state.speedMetersPerSecond, state.angle.getDegrees());
  }

  /**
   * getEncoder:
   * 
   * @return Return the module angle in degrees 0-360. CCW positive.
   *         Straight Forward should be 0 (with the addjustment of module offset)
   */
  public double getRelEncoderRotations() {
    return angleMotor.getPosition().getValueAsDouble();
  }

  public double getRelativeEncoderDeg() {
    return angleMotor.getPosition().getValueAsDouble() * 360.0;
  }


  public TalonFX getAngleMotor() {
    return angleMotor;
  }


  /*
   * Return the applied voltage on the drive motor (0-12V)
   */
  public double getDriveOutput() {
    return speedMotor.getMotorOutputStatus().getValueAsDouble();
  }

  public double getVelocity() {
    return speedMotor.getVelocity().getValueAsDouble();
  }

  /*
   * Return a rotation object for the module absolute encoder.
   */
  private Rotation2d getRotation() {
    return new Rotation2d(absoluteEncoder.getAbsPosition()*2*Math.PI);
  }

  /*
   * Return the absolute encoder position in radians (0-2pi)
   */
  public double getEncoderRadians() {
    return (absoluteEncoder.getAbsPosition()*2*Math.PI);
  }

  /*
   * What is the position of the module using the encoder information.
   * The encoder cinfiguration should be set so that this function will
   * return the valid location in meters.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(speedMotor.getPosition().getValueAsDouble(), getRotation());
  }

  /*
   * Another view of the module state, showing velocity instead of position
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(speedMotor.getVelocity().getValueAsDouble(), getRotation());
  }
}
