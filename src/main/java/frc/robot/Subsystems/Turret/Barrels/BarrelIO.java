package frc.robot.Subsystems.Turret.Barrels;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface BarrelIO {
  @AutoLog
  public static class BarrelIOInputs {
    public boolean barrelConnected = false;
    public double barrelPositionRad = 0.0;
    public double barrelVelocityRadPerSec = 0.0;
    public double barrelAppliedVolts = 0.0;
    public double barrelCurrentAmps = 0.0;
  }

  public default void updateInputs(BarrelIOInputs inputs) {}

  public default void setOpenLoop(double voltage) {}

  public default void setBarrelPos(Rotation2d radians, double arbFF) {}
}
