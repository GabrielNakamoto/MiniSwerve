// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.modulePositions;

import static edu.wpi.first.units.Units.*;

import java.util.function.IntPredicate;

import javax.lang.model.element.ModuleElement;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.opencv.dnn.Model;
import org.w3c.dom.events.MouseEvent;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class Module {
    private final ModuleIO io;
    private final Translation2d chassisPosition;
    private final double radius;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    public record ModuleStateObservation(
        double drivePosition, Rotation2d yaw, double timestamp) {};

    public Module(ModuleIO io, int module) {
        index = module;
        this.io = io;
        this.chassisPosition = modulePositions[module];
        this.radius = chassisPosition.getNorm();
    }

    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Drive/module" + Integer.toString(index), inputs);
    }

    public ModuleStateObservation[] getOdometryMeasurements() {
        ModuleStateObservation[] observations = new ModuleStateObservation[inputs.odometryTimestamps.length];

        for (int i = 0; i < inputs.odometryTimestamps.length; ++i) {
            observations[i] = new ModuleStateObservation(
                inputs.odometryDrivePositions[i],
                inputs.odometryTurnPositions[i],
                inputs.odometryTimestamps[i]);
        }
        return observations;
    }

    public void runSetpoint(double vx, double vy, Rotation2d omega, Rotation2d driveYaw) {
        // Translation2d fieldRelVelocity = new Translation2d(vx, vx)
       //     .rotateBy(driveYaw);
        var state = fromChassisVelocity(
            vx,
            vy,
            omega,
            driveYaw);

        // m/s -> rpm
        // double driveVelocity = (state.getTranslation().getNorm() / (2 * Math.PI * radius)) * 60;
        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/requestedVelocity", state.getTranslation().getNorm());
        this.io.runDriveVelocity(state.getTranslation().getNorm());
        this.io.runTurnAngle(state.getRotation());
    }

    public void runTurnVoltage(Voltage requested) {
        io.runTurnVoltage(requested);
    }

    public void runDriveVoltage(Voltage requested) {
        io.runDriveVoltage(requested);
    }

    public Rotation2d getTurnAngle() {
        return inputs.turnPosition;
    }

    public double getDriveVelocityMps() {
        return inputs.driveVelocityRadsPerSecond * DriveConstants.moduleWheelRadius.in(Meters);
    }

    public Transform2d fromChassisVelocity(double vx, double vy, Rotation2d omega, Rotation2d driveYaw){
        Translation2d translationVelocity = new Translation2d(vx, vy);
        // translationVelocity = translationVelocity.rotateBy(driveYaw); // is this right??
        double vtScalar = omega.getRadians() * radius;

        // clockwise or counterclockwise?
        // assuming counterclockwise
        Translation2d tangentVelocity = new Translation2d(-chassisPosition.getY(), chassisPosition.getX())
            .div(radius)
            .times(vtScalar);

        Translation2d moduleVelocity = translationVelocity.plus(tangentVelocity);

        // TODO: optimize angle
        return new Transform2d(moduleVelocity, moduleVelocity.getAngle());
    }
}
