package frc.robot.Subsystems.Turret.Elevation;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevationIO {

  @AutoLog
  public static class ElevationIOInputs {
    public boolean elevationConnected = false;
    public double elevationPositionRad = 0.0;
    public double elevationVelocityRadPerSec = 0.0;
    public double elevationAppliedVolts = 0.0;
    public double elevationCurrentAmps = 0.0;

    public boolean elevationTwoConnected = false;
    public double elevationTwoPositionRad = 0.0;
    public double elevationTwoVelocityRadPerSec = 0.0;
    public double elevationTwoAppliedVolts = 0.0;
    public double elevationTwoCurrentAmps = 0.0;

    public boolean limitSwitchHit = false;
  }

  public default void updateInputs(ElevationIOInputs inputs) {}

  public default void setElevationOpenLoop(double voltage) {}

  public default void setElevationPos(Rotation2d angle, double arbFF) {}
}
