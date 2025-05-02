// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveDriveSimulation swerveDriveSimulation =
    new SwerveDriveSimulation(Drive.mapleSwerveConfig, new Pose2d(3, 3, Rotation2d.kZero));
  private final Drive drive;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        drive = new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
      }
      case SIM -> {
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        drive = new Drive(
          new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
          new ModuleIOSim(swerveDriveSimulation.getModules()[0], 0),
          new ModuleIOSim(swerveDriveSimulation.getModules()[1], 1),
          new ModuleIOSim(swerveDriveSimulation.getModules()[2], 2),
          new ModuleIOSim(swerveDriveSimulation.getModules()[3], 3));
      }
      default -> {
        drive = new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
      }
    }
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DoubleSupplier inputX = () -> m_driverController.getLeftY() * -1;
    DoubleSupplier inputY = () -> m_driverController.getLeftX() * -1;
    DoubleSupplier inputOmega = () -> -m_driverController.getRightX();

    drive.setDefaultCommand(
      DriveCommands.driveWithJoysticks(drive, inputX, inputY, inputOmega));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return DriveCommands.driveWithJoysticks(drive, () -> -1.0, () -> -0.25, () -> 0.0);
    // return Commands.run(() -> drive.runVelocity(Drive.getMaximumLinearSpeedMetersPerSec(),0.0, Rotation2d.fromRadians(0.85 * Drive.getMaximumAngularSpeedRadPerSec())), drive);
  }

  public void resetSimulationField() {
    swerveDriveSimulation.setSimulationWorldPose(new Pose2d(1.5, 1.5, Rotation2d.kZero));
    SimulatedArena.getInstance().resetFieldForAuto();
    RobotState.getInstance().setOdometryPose(new Pose2d(1.5, 1.5, Rotation2d.kZero));
  }

  public void logSimObjects() {
    Logger.recordOutput("FieldSimulation/Algae", 
      SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", 
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput("FieldSimulation/RobotPose",
      swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/DriveSpeed", swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
  }
}
