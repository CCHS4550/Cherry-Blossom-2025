package frc.robot.Subsystems.Turret.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticsIO {
  
  //this is largely empty because to be frank I don't know what inputs are even important to log and we will never use pneumatics in a real comp, nor will any good teams, so there aren't any real references
  @AutoLog
  public static class PneumaticsIOInputs {
    public boolean connected = false;
    public double pressurePSI;
    //public double pressureChangePSIPerSecond = 0.0;

    //public double[] pressureTimestamps = new double[] {};
    //public double[] pressureValues = new double[] {};

    public double compressorAppliedVolts;
    public double compressorCurrentAmps;
  }

  public default void updateInputs(PneumaticsIOInputs inputs) {}

  public default void setCompressor(double percent) {}

  public default void enablePressureSeal() {}

  public default void disablePressureSeal() {}

  public default void setShootingSeal(boolean direction){}

  public default int getPressure(){return 0;}
}
