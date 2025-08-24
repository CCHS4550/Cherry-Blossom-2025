package frc.robot.Subsystems.Turret.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

// creates an interface for hardware to fully define
public interface PneumaticsIO {

  // this is largely empty because to be frank I don't know what inputs are even important to log
  // and we will never use pneumatics in a real comp, nor will any good teams, so there aren't any
  // real references
  
  // automatically generates a class using clonable that logs the following variables with advantage
  // scope
  @AutoLog
  public static class PneumaticsIOInputs {
    public boolean connected = false;
    public double pressurePSI;
    // public double pressureChangePSIPerSecond = 0.0;

    // public double[] pressureTimestamps = new double[] {};
    // public double[] pressureValues = new double[] {};

    public double compressorAppliedVolts;
    public double compressorCurrentAmps;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PneumaticsIOInputs inputs) {}

  /** Run the compressor at the specified percent. */
  public default void setCompressor(double percent) {}

  /** Turn the pressure solenoid on. */
  public default void enablePressureSeal() {}

  /** Turn the pressure solenoid off*/
  public default void disablePressureSeal() {}

  /** Turn the shooting solenoid in a direction */
  public default void setShootingSeal(boolean direction) {}

  /** Return what pressure the tank is at. */
  public default int getPressure() {
    return 0;
  }
}
