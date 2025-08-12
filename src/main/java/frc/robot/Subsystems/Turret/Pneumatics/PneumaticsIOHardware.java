package frc.robot.Subsystems.Turret.Pneumatics;

import static frc.robot.Util.SparkUtil.*;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Util.SparkUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PneumaticsIOHardware implements PneumaticsIO {
  private final SparkBase compressor;
  private final SparkAnalogSensor transducer;
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  private final Compressor compressorFan;
  private final Debouncer compressorDebouncer = new Debouncer(0.5);

  private final DoubleSolenoid pressureSeal;
  private final Solenoid shootingSeal;

  public PneumaticsIOHardware() {
    compressor = new SparkMax(Constants.PneumaticConstants.compressorCanID, MotorType.kBrushed);
    compressorFan =
        new Compressor(Constants.PneumaticConstants.compressorFanPort, PneumaticsModuleType.REVPH);
    pressureSeal =
        new DoubleSolenoid(
            Constants.PneumaticConstants.compressorFanPort,
            PneumaticsModuleType.REVPH,
            Constants.PneumaticConstants.pressureSealForward,
            Constants.PneumaticConstants.pressureSealBackward);
    shootingSeal =
        new Solenoid(
            Constants.PneumaticConstants.compressorFanPort,
            PneumaticsModuleType.REVPH,
            Constants.PneumaticConstants.shootingSeal);

    var compressorConfig = new SparkMaxConfig();
    compressorConfig.inverted(Constants.PneumaticConstants.compressorInverted);
    compressorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
    SparkUtil.makeItWork(
        compressor,
        5,
        () ->
            compressor.configure(
                compressorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    transducer = compressor.getAnalog();

    // pneumatics functions must be set to an inital value for certain functions like toggle to work
    pressureSeal.set(Value.kReverse);
    shootingSeal.set(false);
    compressorFan.disable();
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    inputs.connected = compressorDebouncer.calculate(!SparkUtil.stickyFault);
    inputs.pressurePSI = getPressure();
    ifOK(
        compressor,
        new DoubleSupplier[] {compressor::getAppliedOutput, compressor::getBusVoltage},
        (values) -> inputs.compressorAppliedVolts = values[0] * values[1]);
    ifOk(compressor, compressor::getOutputCurrent, (value) -> inputs.compressorCurrentAmps = value);
  }

  @Override
  public void setCompressor(double percent) {
    compressor.setVoltage(percent * 12);

    if (percent > 0) {
      compressorFan.enableDigital();
    } else {
      compressorFan.disable();
    }
  }

  @Override
  public void enablePressureSeal() {
    pressureSeal.set(Value.kForward);
  }

  @Override
  public void disablePressureSeal() {
    pressureSeal.set(Value.kReverse);
  }

  @Override
  public void setShootingSeal(boolean direction) {
    shootingSeal.set(direction);
  }

  @Override
  public int getPressure() {

    /* Found by graphing the transducer voltage against the read psi on the pressure gauge. Graph and find the Linear regression. Courtesy of Dr. Harrison's Physics Class */
    int psi = (int) ((54 * transducer.getVoltage()) - 12.2);
    Logger.recordOutput("Transducer Voltage", transducer.getVoltage());
    Logger.recordOutput("Unfiltered PSI", psi);
    psi = (int) filter.calculate(psi);
    Logger.recordOutput("Filtered PSI", psi);
    return psi;
    // SmartDashboard.putNumber("Transducer Voltage", transducer.getVoltage());

  }
}
