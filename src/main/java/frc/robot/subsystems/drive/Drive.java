// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.bumperLength;
import static frc.robot.subsystems.drive.DriveConstants.bumperWidth;
import static frc.robot.subsystems.drive.DriveConstants.modulePositions;
import static frc.robot.subsystems.drive.DriveConstants.robotMass;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.RobotState;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.subsystems.drive.Module.ModuleStateObservation;

public class Drive extends SubsystemBase {
  public static final DriveTrainSimulationConfig mapleSwerveConfig = DriveTrainSimulationConfig.Default()
    .withBumperSize(bumperLength, bumperWidth)
    .withRobotMass(robotMass)
    .withCustomModuleTranslations(modulePositions)
    .withGyro(COTS.ofPigeon2())
    .withSwerveModule(new SwerveModuleSimulationConfig(
      DCMotor.getNeo550(1),
      DCMotor.getNeo550(1),
      6.746031746031747,
      21.428571428571427,
      Volts.of(0.2),
      Volts.of(0.2),
      Inches.of(1.967),
      KilogramSquareMeters.of(0.01),
      1.2));

  public static final Lock odometryLock = new ReentrantLock();
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  /** Creates a new Drive. */
  public Drive(
    GyroIO gyroIO,
    ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO){

    this.gyroIO = gyroIO;

    this.modules[0] = new Module(flModuleIO, 0);
    this.modules[1] = new Module(frModuleIO, 1);
    this.modules[2] = new Module(blModuleIO, 2);
    this.modules[3] = new Module(brModuleIO, 3);
  }

  public static double getMaximumLinearSpeedMetersPerSec() {
    return 6.7;
  }

  public static double getMaximumAngularSpeedRadPerSec() {
    return 10.0;
  }

  public void runVelocity(double vx, double vy, Rotation2d omega) {
    runVelocity(new Translation2d(vx, vy), omega);
  }

  public void runVelocity(Translation2d vl, Rotation2d omega) {
    Rotation2d driveYaw = gyroInputs.yawPosition;

    Logger.recordOutput("Drive/Requested/Omega", omega);
    Logger.recordOutput("Drive/Requested/Linear", vl);

    if (DriverStation.isEnabled() && DriverStation.getAlliance().get() == Alliance.Red)
      driveYaw = driveYaw.plus(Rotation2d.k180deg);

    vl.rotateBy(getYaw());
    for (Module module : modules) {
        module.runSetpoint(vl.getX(), vl.getY(), omega, gyroInputs.yawPosition);
    }
  }

  public void runTurnVoltage(Voltage requested) {
    for (Module module : modules) {
      module.runTurnVoltage(requested);
    }
  }

  public void runDriveVoltage(Voltage requested) {
    for (Module module : modules) {
      module.runDriveVoltage(requested);
    }
  }

  public Rotation2d getYaw() {
    return gyroInputs.yawPosition;
  }

  public Translation2d getDriveVelocity(){
    Translation2d robotVelocity = Translation2d.kZero;

    for (Module module : modules) {
      Translation2d moduleVelocity = new Pose2d(Translation2d.kZero, module.getTurnAngle())
        .transformBy(new Transform2d(module.getDriveVelocityMps(), 0.0, Rotation2d.kZero)).getTranslation();
      robotVelocity = robotVelocity.plus(moduleVelocity);
    }
    return robotVelocity.div(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (Module module : modules) {
      module.periodic();
    }
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    Logger.recordOutput("Drive/driveVelocity", getDriveVelocity().getNorm());

    int sampleCount = gyroInputs.odometryYawTimestamps.length;

    ModuleStateObservation[][] moduleObservations  = new ModuleStateObservation[4][sampleCount];
    for (int j = 0; j < 4; ++j) {
      moduleObservations[j] = modules[j].getOdometryMeasurements();
    }

    for (int i = 0; i < sampleCount; ++i) {
      ModuleStateObservation[] moduleStates = new ModuleStateObservation[4];
      for (int j = 0; j < 4; ++j) {
        moduleStates[j] = moduleObservations[j][i];
      }
      Rotation2d gyroYaw = gyroInputs.odometryYawPositions[i];

      RobotState.getInstance().addOdometryObservation(
        new OdometryObservation(
          moduleStates,
          Optional.of(gyroYaw),
          gyroInputs.odometryYawTimestamps[i]));
    }
  }
}
