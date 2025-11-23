// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directsory of this project.

package swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

// copy pasted 

import com.ctre.phoenix6.hardware.TalonFX; // For the Talon FX device class
import com.ctre.phoenix6.controls.*;      // For control requests (DutyCycleOut, PositionVoltage, etc.)
import com.ctre.phoenix6.configs.*;        // For configuration objects (TalonFXConfiguration, etc.)
import com.ctre.phoenix6.signals.*;        // For signals and enums (InvertedValue, NeutralMode, etc.)
import com.ctre.phoenix6.StatusCode;       // For checking API status
import com.ctre.phoenix6.CANBus;         // For general CAN bus status
import com.ctre.phoenix6.mechanisms.swerve.*; // For swerve drive specific classes
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class SwerveModule {
    private static final double gearRatio = 150.0; // check this number
    private TalonFX angleMotor;
    private TalonFX speedMotor;
    private PIDController pidController;
    private double maxVelocity;
    private double maxVoltage;
  
    private double WHEEL_DIAMETER = 8;
    private double wheelCirc = Math.PI * WHEEL_DIAMETER;
    private double driveReduction = 1.0 / 6.75;
    
  
    public SwerveModule(int angleMotorId, int speedMotorId, boolean driveMotorReversed, boolean angleMotorReversed,
        boolean angleEncoderReversed, double angleEncoderConversionFactor, double angleEncoderOffset,
        double maxVelocity, double maxVoltage) {
      this.angleMotor = new TalonFX(angleMotorId);
      this.speedMotor = new TalonFX(speedMotorId);
  
      //this.angleMotor.restoreFactoryDefaults();
      //this.speedMotor.restoreFactoryDefaults();
  
      this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
     
      this.maxVelocity = maxVelocity;
      this.maxVoltage = maxVoltage;
      
  
      this.pidController.enableContinuousInput(-180, 180);
  
      double driveReduction = 1.0 / 6.75;
      double WHEEL_DIAMETER = 0.1016;
    
  
      TalonFXConfiguration angleConfig = new TalonFXConfiguration();
      TalonFXConfiguration driveConfig = new TalonFXConfiguration();
      
      // angle
      angleConfig.MotorOutput.Inverted =
          angleMotorReversed ? InvertedValue.Clockwise_Positive
                             : InvertedValue.CounterClockwise_Positive;
      
      // drive
      driveConfig.MotorOutput.Inverted =
          driveMotorReversed ? InvertedValue.Clockwise_Positive
                             : InvertedValue.CounterClockwise_Positive;
      
      angleMotor.getConfigurator().apply(angleConfig);
      speedMotor.getConfigurator().apply(driveConfig);
      //angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
      //speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
  
      //this.encoder.setZeroOffset(angleEncoderOffset);
  

  
    }
  
    
  
    public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
      this(config.angleMotorId,
          config.driveMotorId,
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
      double angle_voltage = pidController.calculate(this.getEncoder(), angle);
  
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
     * 
     *       
     */
  
    public double getEncoder() {
    return angleMotor.getPosition().getValueAsDouble() * (360.0 / gearRatio);
  }



  /*
   * Return the applied voltage on the drive motor (0-12V)
   */
  public double getDriveOutput() {
    double dutyCycle = speedMotor.getDutyCycle().getValueAsDouble();
    double busVoltage = speedMotor.getSupplyVoltage().getValueAsDouble();

    
    return dutyCycle * busVoltage; 
  }

  /*
   * Return a rotation object for the module absolute encoder.
   */
  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getEncoder());
  }

  /*
   * Return the absolute encoder position in radians (0-2pi)
   */
  public double getEncoderRadians() {
    return Units.degreesToRadians(getEncoder());
  }

  /*
   * What is the position of the module using the encoder information.
   * The encoder cinfiguration should be set so that this function will
   * return the valid location in meters.
   */
  public SwerveModulePosition getPosition() {
    double driveRotations = speedMotor.getPosition().getValueAsDouble();
    double driveMeters = driveRotations * wheelCirc * driveReduction;

    return new SwerveModulePosition(driveMeters, getRotation());


  }

  /*
   * Another view of the module state, showing velocity instead of position
   */
  public SwerveModuleState getState() {
  double driveRPS = speedMotor.getVelocity().getValueAsDouble();
  double driveMPS = driveRPS * wheelCirc * driveReduction;

  return new SwerveModuleState(driveMPS, getRotation());
  }
}
