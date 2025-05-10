package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.*;

public class DriveCommands {
    public static Translation2d getLinearVelocityFromJoysticks(double x, double y){
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d heading = new Rotation2d(Math.atan2(y, x));

        linearMagnitude = linearMagnitude * linearMagnitude;

        return new Pose2d(0, 0, heading)
            .transformBy(new Transform2d(linearMagnitude, 0, Rotation2d.kZero))
            .getTranslation();
    }

    public static Command driveWithJoysticks(
        Drive drive,
        DoubleSupplier inputX,
        DoubleSupplier inputY,
        DoubleSupplier inputOmega) {
        // field relative velocity input from driver
        return Commands.run(() -> {
            Translation2d fieldVelocity = getLinearVelocityFromJoysticks(
                inputX.getAsDouble(), inputY.getAsDouble());
            

            double omega = MathUtil.applyDeadband(inputOmega.getAsDouble(), 0.1);

            omega = Math.copySign(omega * omega, omega);

            fieldVelocity = fieldVelocity.times(Drive.getMaximumLinearSpeedMetersPerSec());
            omega = omega * Drive.getMaximumAngularSpeedRadPerSec();

            /*
            boolean isFlipped = DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red;

            Rotation2d driveRotation = drive.getRotation()
                .plus(isFlipped ? new Rotation2d(Math.PI) : Rotation2d.kZero);

            fieldVelocity = fieldVelocity.rotateBy(driveRotation.unaryMinus());*/

            drive.runVelocity(fieldVelocity, omega, true);
        }, drive);
    }    

    public static Command feedForwardCharacterization(Drive drive) {
        Timer timer = new Timer();
        return Commands.run(() -> {
            drive.runDriveVoltage(Volts.of(timer.get() * 0.5));
        }, drive)
            .until(() -> drive.getDriveVelocity().getNorm() > 1e-6)
            .beforeStarting(() -> timer.start())
            .finallyDo(() -> timer.stop())
        .andThen(Commands.run(() -> drive.runDriveVoltage(Volts.of(6.0)))
            .withDeadline(Commands.waitSeconds(5.0)))
        .finallyDo(() -> {
            double kv = 6.0 / drive.getDriveVelocity().getNorm() / DriveConstants.moduleWheelRadius.in(Meters);
            System.out.println("ks = " + timer.get() * 0.5 / DriveConstants.moduleWheelRadius.in(Meters) + "\nkv = " + kv);
            timer.reset();
        });
    }
}

