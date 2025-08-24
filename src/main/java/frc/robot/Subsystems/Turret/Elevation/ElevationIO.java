package frc.robot.Subsystems.Turret.Elevation;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware to fully define
public interface ElevationIO {

  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class ElevationIOInputs {
    // tracking variables for one elevation motor
    public boolean elevationConnected = false;
    public double elevationPositionRad = 0.0;
    public double elevationVelocityRadPerSec = 0.0;
    public double elevationAppliedVolts = 0.0;
    public double elevationCurrentAmps = 0.0;

    // tracking variables for the other elevation motor
    public boolean elevationTwoConnected = false;
    public double elevationTwoPositionRad = 0.0;
    public double elevationTwoVelocityRadPerSec = 0.0;
    public double elevationTwoAppliedVolts = 0.0;
    public double elevationTwoCurrentAmps = 0.0;

    // tracking the limit swithc
    public boolean limitSwitchHit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevationIOInputs inputs) {}

  /** Run the elevation motor at the specified open loop value. */
  public default void setElevationOpenLoop(double voltage) {}

  /** Run the elevation motor to the specified angle. */
  public default void setElevationPos(Rotation2d angle, double arbFF) {}
}
