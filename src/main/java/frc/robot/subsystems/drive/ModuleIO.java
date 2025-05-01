// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double driveVelocityRadsPerSecond = 0.0;
        public double drivePositionRad = 0.0;
        public double turnVelocityRadsPerSecond = 0.0;
        public Rotation2d turnPosition = new Rotation2d(); 

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositions = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    default void updateInputs(ModuleIOInputs inputs) {}
    default void runDriveVelocity(double setpoint) {}
    default void runTurnAngle(Rotation2d setpoint) {}
    default void runDriveVoltage(Voltage requested) {}
    default void runTurnVoltage(Voltage requested) {}
}
