package frc.robot.Subsystems.Turret.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware to fully define
public interface RotationIO {
  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class RotationIOInputs {
    // tracking variables for the rotation motor
    public boolean rotationConnected = false;
    public double rotationPositionRad = 0.0;
    public double rotationVelocityRadPerSec = 0.0;
    public double rotationAppliedVolts = 0.0;
    public double rotationCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(RotationIOInputs inputs) {}

  /** Run the rotation motor at the specified open loop value. */
  public default void setRotationOpenLoop(double voltage) {}

  /** Run the barrel motor to the specified rotation. */
  public default void setRotationPos(Rotation2d angle) {}

  /** Run the barrel motor to the specified rotation with feedforward. */
  public default void setRotationPos(Rotation2d angle, double arbFF) {}
}
