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

// define a class that uses the interface PnuematicsIO
// used to initate the hardware used and define interface methods
// I really don't know how to work with pneumatics so don't trust
public class PneumaticsIOHardware implements PneumaticsIO {
  
  // initiate compressor and psi tracking
  private final SparkBase compressor;
  private final SparkAnalogSensor transducer;
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  // initiate compressor fan 
  private final Compressor compressorFan;
  
  // track if connected
  private final Debouncer compressorDebouncer = new Debouncer(0.5);

  // solenoids for pressure and shooting
  private final DoubleSolenoid pressureSeal;
  private final Solenoid shootingSeal;

  public PneumaticsIOHardware() {
    
    //create the compressor and compressor fan
    compressor = new SparkMax(Constants.PneumaticConstants.compressorCanID, MotorType.kBrushed);
    compressorFan =
        new Compressor(Constants.PneumaticConstants.compressorFanPort, PneumaticsModuleType.REVPH);
    
    // create the pressure solenoid
    pressureSeal =
        new DoubleSolenoid(
            Constants.PneumaticConstants.compressorFanPort,
            PneumaticsModuleType.REVPH,
            Constants.PneumaticConstants.pressureSealForward,
            Constants.PneumaticConstants.pressureSealBackward);

    // create the shooting solenoid 
    shootingSeal =
        new Solenoid(
            Constants.PneumaticConstants.compressorFanPort,
            PneumaticsModuleType.REVPH,
            Constants.PneumaticConstants.shootingSeal);

    // configure the compressor
    var compressorConfig = new SparkMaxConfig();
    
    // set the inverted
    compressorConfig.inverted(Constants.PneumaticConstants.compressorInverted);
    
    /**
     * idleMode is Brake, stay at position when stopped
     * voltage compensation = 12 because
     * working with 12v car battery
     */
    compressorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12);
    
    /** configures the motor, retrying if faulting */
    SparkUtil.makeItWork(
        compressor,
        5,
        () ->
            compressor.configure(
                compressorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // set the transducer for PSI tracking
    transducer = compressor.getAnalog();

    // pneumatics functions must be set to an inital value for certain functions like toggle to work
    pressureSeal.set(Value.kReverse);
    shootingSeal.set(false);
    compressorFan.disable();
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    // if compressor is connected
    inputs.connected = compressorDebouncer.calculate(!SparkUtil.stickyFault);
    
    // what the PSI is
    inputs.pressurePSI = getPressure();
    
    // update compressor values, only accepting if no sticky fault present
    ifOK(
        compressor,
        new DoubleSupplier[] {compressor::getAppliedOutput, compressor::getBusVoltage},
        (values) -> inputs.compressorAppliedVolts = values[0] * values[1]);
    ifOk(compressor, compressor::getOutputCurrent, (value) -> inputs.compressorCurrentAmps = value);
  }

  /**
   * set the compressor to a desired percent
   * 
   * @param percent the desired percent to set to
   */
  @Override
  public void setCompressor(double percent) {
    compressor.setVoltage(percent * 12);

    if (percent > 0) {
      compressorFan.enableDigital();
    } else {
      compressorFan.disable();
    }
  }

  /**
   * create a seal with the barrel
   */
  @Override
  public void enablePressureSeal() {
    pressureSeal.set(Value.kForward);
  }

  /**
   * remove seal with the barrel
   */
  @Override
  public void disablePressureSeal() {
    pressureSeal.set(Value.kReverse);
  }

  /**
   * make the shooting seal go in a direction
   * 
   * @param direction if the seal should go on(true) or off(false)
   */
  @Override
  public void setShootingSeal(boolean direction) {
    shootingSeal.set(direction);
  }

  /**
   * @return the pressure in PSI of the pneumatics subsystem
   */
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
