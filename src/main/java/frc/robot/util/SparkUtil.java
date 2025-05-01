package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj.Timer;

public class SparkUtil {
    public static double[] getSimulatedOdometryTimestamps() {
        // timestamp for each subtick during the loop cycle
        final double[] odometryTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimestamps.length; i++) {
            // loop cycle start time + evenly distributed subticks for each timestamp
            odometryTimestamps[i] = Timer.getFPGATimestamp()
                - 0.02
                + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimestamps;
    } 
}
