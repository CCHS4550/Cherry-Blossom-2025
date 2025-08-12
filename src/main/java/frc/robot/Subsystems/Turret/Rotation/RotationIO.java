package frc.robot.Subsystems.Turret.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface RotationIO {

  @AutoLog
  public static class RotationIOInputs {
    public boolean rotationConnected = false;
    public double rotationPositionRad = 0.0;
    public double rotationVelocityRadPerSec = 0.0;
    public double rotationAppliedVolts = 0.0;
    public double rotationCurrentAmps = 0.0;
  }

  public default void updateInputs(RotationIOInputs inputs) {}

  public default void setRotationOpenLoop(double voltage) {}

  public default void setRotationPos(Rotation2d angle) {}

  public default void setRotationPos(Rotation2d angle, double arbFF) {}
}
