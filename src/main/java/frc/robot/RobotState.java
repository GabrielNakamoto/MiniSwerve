package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.modulePositions;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.commands.DriveCommands;
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
        odometryPose = pose;
    }

    public void addOdometryObservation(OdometryObservation observation) {
        if (lastObservation == null) {
            lastObservation = observation;
            return;
        }

        Translation2d robotDisplacement = Translation2d.kZero;
        for (int i = 0; i < 4; ++i) {
            double moduleDisplacement = observation.modulePositions[i].drivePosition() -
                lastObservation.modulePositions[i].drivePosition();

            // convert from radians to meters
            moduleDisplacement = moduleDisplacement * DriveConstants.moduleWheelRadius.in(Meters);

            robotDisplacement = robotDisplacement.plus(new Pose2d(0.0, 0.0, lastObservation.modulePositions[i].yaw())
                .transformBy(new Transform2d(moduleDisplacement, 0.0, Rotation2d.kZero)).getTranslation());
        }
        robotDisplacement = robotDisplacement.div(4)
            .rotateBy(lastObservation.gyroAngle.get());

        this.odometryPose = new Pose2d(odometryPose.plus(new Transform2d(robotDisplacement, Rotation2d.kZero))
            .getTranslation(), observation.gyroAngle().get());

        lastObservation = observation;
    }

    public record OdometryObservation(
        ModuleStateObservation[] modulePositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
}
