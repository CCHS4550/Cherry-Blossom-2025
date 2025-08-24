package frc.robot.Subsystems.Turret.Barrels;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware to fully define
public interface BarrelIO {
  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class BarrelIOInputs {
    // tracking variables for the barrel motor
    public boolean barrelConnected = false;
    public double barrelPositionRad = 0.0;
    public double barrelVelocityRadPerSec = 0.0;
    public double barrelAppliedVolts = 0.0;
    public double barrelCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(BarrelIOInputs inputs) {}

  /** Run the barrel motor at the specified open loop value. */
  public default void setOpenLoop(double voltage) {}

  /** Run the barrel motor to the specified rotation. */
  public default void setBarrelPos(Rotation2d radians, double arbFF) {}
}
