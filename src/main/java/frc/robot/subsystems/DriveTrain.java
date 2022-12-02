// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private static final double DRIVE_SETPOINT_MAX = 0.0;
  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;

  // create telemetry first
  private final SwerveDrive swerve = getSwerve();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    swerve.setFieldOriented(true);
    zeroAzimuths();
  }

  public void zeroAzimuths() {
    swerve.zeroAzimuthEncoders();
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = gyro.getAngle() % 360;
    gyro.setAngleAdjustment(-adj);
  }

  // Swerve Configuration
  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
    // NOTE: ensure encoders are in-phase with motor direction. Encoders should
    // increase
    // when azimuth motor runs in forward direction.
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;
    azimuthConfig.slot0.kP = 20;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 300.0;
    azimuthConfig.slot0.kF = 0.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.motionCruiseVelocity = 800;
    azimuthConfig.peakOutputForward = 0.75;
    azimuthConfig.peakOutputReverse = -0.75;

    TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
    driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    driveConfig.continuousCurrentLimit = 40;
    driveConfig.peakCurrentDuration = 0;
    driveConfig.peakCurrentLimit = 0;

    Wheel[] wheels = new Wheel[4];

    for (int i = 0; i < 4; i++) {
      // Rotation Motor === In our case this will be a sparxmax
      TalonSRX azimuthTalon = new TalonSRX(i);
      azimuthTalon.configAllSettings(azimuthConfig);

      TalonSRX driveTalon = new TalonSRX(i + 10);
      driveTalon.configAllSettings(driveConfig);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      Wheel wheel = new Wheel(azimuthTalon, driveTalon, DRIVE_SETPOINT_MAX);
      wheels[i] = wheel;
    }

    return wheels;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
