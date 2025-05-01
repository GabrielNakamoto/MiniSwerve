package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import frc.robot.util.SparkUtil;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    } 

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity()
            .in(RadiansPerSecond);

        inputs.odometryYawTimestamps = SparkUtil.getSimulatedOdometryTimestamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}
