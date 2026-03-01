package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class FuelDetection {

  /** X position of the detected Fuel in normalized camera space (-1 to +1). */
  public final double x;

  /** Y position of the detected Fuel in normalized camera space (-1 to +1). */
  public final double y;

  /** Radius or size of the detected Fuel blob (pixels or normalized units). */
  public final double size;

  /** Timestamp (in seconds) when the frame was captured. */
  public final double timestamp;

  /** Whether this detection is considered “high confidence.” */
  public final boolean highConfidence;

  public final BiConsumer<FuelDetection, Double> fuelConsumer =
      (detection, timestamp) -> {
        Logger.recordOutput("Vision/Fuel/LatestDetection", detection.toString());
        Logger.recordOutput("Vision/Fuel/LatestTimestamp", timestamp.toString());
      };

  public FuelDetection(double x, double y, double size, double timestamp, boolean highConfidence) {
    this.x = x;
    this.y = y;
    this.size = size;
    this.timestamp = timestamp;
    this.highConfidence = highConfidence;
  }

  @Override
  public String toString() {
    return String.format(
        "FuelDetection[x=%.3f, y=%.3f, size=%.3f, time=%.3f, highConf=%b]",
        x, y, size, timestamp, highConfidence);
  }
}
