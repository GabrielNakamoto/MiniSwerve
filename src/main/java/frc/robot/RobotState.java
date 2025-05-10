package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Module.ModuleStateObservation;

public class RobotState {
    private static RobotState instance = null; 

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private RobotState() {
    }

    @AutoLogOutput(key="RobotState/odometryPose")
    private Pose2d odometryPose = Pose2d.kZero;

    public OdometryObservation lastObservation = null;

    public Pose2d getEstimatedDrivePose() {
        return odometryPose;
    }

    public void setOdometryPose(Pose2d pose) {
        System.out.println("Reset odometry");
        odometryPose = pose;
        lastObservation = null;
    }

    public void addOdometryObservation(OdometryObservation observation) {
        if (lastObservation == null) {
            lastObservation = observation;
            return;
        }

        Translation2d robotDisplacement = Translation2d.kZero;
        for (int i = 0; i < 4; ++i) {
            double arcLength = observation.modulePositions[i].drivePosition() -
                lastObservation.modulePositions[i].drivePosition();

            // convert from radians to meters
            arcLength = arcLength * DriveConstants.moduleWheelRadius.in(Meters);
            double omega = observation.modulePositions[i].yaw().minus(
                lastObservation.modulePositions[i].yaw()).getRadians();

            if (omega < 1e-6) {
                Translation2d linearDisplacement = new Pose2d(0.0, 0.0, lastObservation.modulePositions[i].yaw())
                    .transformBy(new Transform2d(arcLength, 0.0, Rotation2d.kZero)).getTranslation();
                robotDisplacement = robotDisplacement.plus(linearDisplacement);
                continue;
            }

            double radius = arcLength / omega;

            Translation2d lastVector = new Pose2d(0.0, 0.0, Rotation2d.fromRadians(
                lastObservation.modulePositions[i].yaw().getRadians() - (Math.PI / 2)))
                .transformBy(new Transform2d(radius, 0.0, Rotation2d.kZero)).getTranslation();
            Translation2d currentVector = lastVector.rotateBy(Rotation2d.fromRadians(omega));

            Translation2d arcDisplacement = currentVector.minus(lastVector);

            Logger.recordOutput("RobotState/arcLength", arcLength);
            Logger.recordOutput("RobotState/omega", omega);
            Logger.recordOutput("RobotState/radius", radius);
            Logger.recordOutput("RobotState/lastVector", lastVector);
            Logger.recordOutput("RobotState/currentVector", currentVector);
            Logger.recordOutput("RobotState/arcDisplacement", arcDisplacement);

            robotDisplacement = robotDisplacement.plus(arcDisplacement);
        }
        robotDisplacement = robotDisplacement.div(4)
            .rotateBy(lastObservation.gyroAngle.get());


        this.odometryPose = new Pose2d(odometryPose.transformBy(new Transform2d(robotDisplacement, Rotation2d.kZero))
            .getTranslation(), observation.gyroAngle().get());

        lastObservation = observation;
    }

    public record OdometryObservation(
        ModuleStateObservation[] modulePositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
}
