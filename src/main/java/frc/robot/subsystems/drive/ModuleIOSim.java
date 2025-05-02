package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.driveCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.driveSimKd;
import static frc.robot.subsystems.drive.DriveConstants.driveSimKp;
import static frc.robot.subsystems.drive.DriveConstants.driveSimKs;
import static frc.robot.subsystems.drive.DriveConstants.driveSimKv;
import static frc.robot.subsystems.drive.DriveConstants.turnCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.turnSimKd;
import static frc.robot.subsystems.drive.DriveConstants.turnSimKp;

import java.util.Arrays;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.SparkUtil;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSim;

    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private double driveFFVolts = 0.0;

    private final PIDController driveController
        = new PIDController(driveSimKp, 0, driveSimKd);
    private final PIDController turnController
        = new PIDController(turnSimKp, 0, turnSimKd);

    private final int index;

    public ModuleIOSim(SwerveModuleSimulation moduleSim, int index) {
        this.index = index;
        this.moduleSim = moduleSim;
        this.driveMotor = moduleSim
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(driveCurrentLimit));
        this.turnMotor = moduleSim
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(turnCurrentLimit));

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        updateOutputs();

        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/Applied/driveVolts", driveMotor.getAppliedVoltage());
        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/Applied/turnVolts", turnMotor.getAppliedVoltage());

        inputs.driveVelocityRadsPerSecond =
            moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.drivePositionRad = 
            moduleSim.getDriveWheelFinalPosition().in(Radians);

        inputs.turnVelocityRadsPerSecond = 
            moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnPosition = moduleSim.getSteerAbsoluteFacing();

        inputs.odometryTimestamps = SparkUtil.getSimulatedOdometryTimestamps();
        inputs.odometryDrivePositions =
            Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
                .mapToDouble((angle) -> angle.in(Radians)).toArray();
        inputs.odometryTurnPositions =
            moduleSim.getCachedSteerAbsolutePositions();
        
        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/DriveSpeedMPS", inputs.driveVelocityRadsPerSecond * DriveConstants.moduleWheelRadius.in(Meters));
    }

    public void updateOutputs() {
        double driveVoltsRequested = driveFFVolts + driveController.calculate(
            moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        double turnVoltsRequested = turnController.calculate(
            moduleSim.getSteerAbsoluteFacing().getRadians());
        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/Requested/driveVolts", driveVoltsRequested);
        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/Requested/turnVolts", turnVoltsRequested);

        driveMotor.requestVoltage(Volts.of(driveVoltsRequested));
        turnMotor.requestVoltage(Volts.of(turnVoltsRequested));
    }

    @Override
    public void runDriveVoltage(Voltage requested) {
        driveMotor.requestVoltage(requested);
    }

    @Override
    public void runTurnVoltage(Voltage requested) {
        turnMotor.requestVoltage(requested);
    }

    @Override
    public void runDriveVelocity(double driveVelocity) {
        driveFFVolts = driveSimKs * Math.signum(driveVelocity) + driveVelocity * driveSimKv;
        driveController.setSetpoint(driveVelocity);
    }

    @Override
    public void runTurnAngle(Rotation2d rotation) {
        Logger.recordOutput("Drive/module" + Integer.toString(index) + "/Requested/turnAngle", rotation);
        turnController.setSetpoint(rotation.getRadians());
    }
}