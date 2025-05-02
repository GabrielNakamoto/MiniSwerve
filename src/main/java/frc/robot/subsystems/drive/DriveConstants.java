package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class DriveConstants {

    public static final Distance bumperWidth = Inches.of(14); // this is actually without bumpers rn
    public static final Distance bumperLength = Inches.of(14);

    public static final Distance moduleWheelRadius = Inches.of(2.0);

    public static final Mass robotMass = Kilograms.of(40);

    public static final Translation2d[] modulePositions = {
        new Translation2d(
            Units.inchesToMeters(5.98),
            Units.inchesToMeters(-6.336)), // FLS
        new Translation2d(
            Units.inchesToMeters(5.98),
            Units.inchesToMeters(6.336)), // FRS
        new Translation2d(
            Units.inchesToMeters(-5.98),
            Units.inchesToMeters(-6.336)), // BLS
        new Translation2d(
            Units.inchesToMeters(-5.98),
            Units.inchesToMeters(6.336)) // BRS
    };

    public static final Rotation2d[] moduleZeroRotations = {
        new Rotation2d(),
        new Rotation2d(),
        new Rotation2d(),
        new Rotation2d()
    };

    public static final double odometryFrequency = 100; // hz

    // Drive CAN Ids
    public static final int frontLeftDriveCanId = 0; 
    public static final int frontRightDriveCanId = 0; 
    public static final int backLeftDriveCanId = 0; 
    public static final int backRightDriveCanId = 0; 

    // Turn CAN Ids
    public static final int frontLeftTurnCanId = 0; 
    public static final int frontRightTurnCanId = 0; 
    public static final int backLeftTurnCanId = 0; 
    public static final int backRightTurnCanId = 0; 
    
    // Current limits
    public static final int driveCurrentLimit = 100;
    public static final int turnCurrentLimit = 100;
    
    // reduction -> decrease speed, increase torque through gearbox
    public static final double driveGearRatio = 6.746031746031747; 
    public static final double turnGearRatio = 21.428571428571427;

    // Drive Encoder configuration
    public static final double drivePositionEncoderFactor =
        2 * Math.PI * driveGearRatio; // Rotations -> wheel radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60 / turnGearRatio; // Rotor RPM -> wheel rads / sec
    public static final int driveEncoderPeriodMs = 10;
    // depth controls how many samples will be averaged to return a reading
    // noise vs responsiveness
    public static final int driveEncoderDepth = 2;

    // Odometry configuration
    public static final int drivePositionUpdateMs = 20;
    public static final int driveVelocityUpdateMs = 20;
    public static final int driveOutputUpdateMs = 20;
    public static final int driveCurrentUpdateMs = 20;
    public static final int driveBusUpdateMs = 20;

    // Odometry configuration
    public static final int turnPositionUpdateMs = 20;
    public static final int turnVelocityUpdateMs = 20;
    public static final int turnOutputUpdateMs = 20;
    public static final int turnCurrentUpdateMs = 20;
    public static final int turnBusUpdateMs = 20;
    
    // Turn Encoder configuration
    public static final double turnEncoderPositionFactor =
        2 * Math.PI * turnGearRatio;
    public static final double turnEncoderVelocityFactor =
        (2 * Math.PI) / 60 / turnGearRatio;
    public static final int turnEncoderDepth = 2;

    // Real closed loop tuning
    public static final double turnPIDMin = 0.0;
    public static final double turnPIDMax = 2 * Math.PI;

    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;

    public static final double driveKs = 0.0;
    public static final double driveKv = 0.0;

    public static final double turnKp = 0.0;
    public static final double turnKd = 0.0;


    // Sim closed loop tuning
    public static final double driveSimKp = 1.5;
    public static final double driveSimKd = 0.0;

    public static final double driveSimKv = 2.65;
    public static final double driveSimKs = 0.03;

    public static final double turnSimKp = 4.5;
    public static final double turnSimKd = 0.0;
}
