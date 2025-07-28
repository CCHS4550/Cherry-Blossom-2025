package frc.robot.Subsystems.Turret.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    public boolean connected = false;
    public double pressurePSI = 0.0;
    public double pressureChangePSIPerSecond = 0.0;

    public double[] pressureTimestamps = new double[] {};
    public double[] pressureValues = new double[] {};

    public double desiredPressure = 0.0;
    public double desiredPressurePerSecond = 0.0;
  }

  default void updateInputs(PneumaticsIOInputs inputs) {}
}
