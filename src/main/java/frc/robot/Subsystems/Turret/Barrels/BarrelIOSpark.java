package frc.robot.Subsystems.Turret.Barrels;

import static frc.robot.Util.SparkUtil.ifOK;
import static frc.robot.Util.SparkUtil.ifOk;
import static frc.robot.Util.SparkUtil.makeItWork;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Util.SparkUtil;
import java.util.function.DoubleSupplier;

// define a class that uses the interface BarrelIO
// used to initiate the hardware used on the row and define the interface methods
public class BarrelIOSpark implements BarrelIO {

  // the motor and encode of the barrel
  private final SparkBase barrelSpark;
  private final RelativeEncoder barrelEncoder;

  // closed loop control for the barrel
  private final SparkClosedLoopController barrelController;

  // checks if the barrel spark is disconnected
  private final Debouncer barrelDebouncer = new Debouncer(0.5);

  /** constructor for the barrel */
  public BarrelIOSpark() {
    // fully define the sparkmax and encoder
    barrelSpark =
        new SparkMax(
            Constants.MechanismConstants.BarrelConstants.barrelCanID, MotorType.kBrushless);

    barrelEncoder = barrelSpark.getEncoder();

    // declare the motors closed loop control
    barrelController = barrelSpark.getClosedLoopController();

    // config for the barrel motor
    var barrelConfig = new SparkMaxConfig();

    // if it should be inverted
    barrelConfig.inverted(Constants.MechanismConstants.BarrelConstants.barrelInverted);

    /**
     * idleMode is Brake, stay at position when stopped set the smart current limit to avoid going
     * over what the motor can handle voltage compensation = 12 because working with 12v car battery
     */
    barrelConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.BarrelConstants.barrelCurrentLimit)
        .voltageCompensation(12.0);

    /**
     * configures the encoder position factor converts rotations to radians while accounting for any
     * gearing
     *
     * <p>velocity factor converts rotations/min to radians/sec while accounting for any gearing
     *
     * <p>this is now automatically applied anytime we request motor information
     *
     * <p>average depth is the bit size of the sampling depth, must be a power of 2
     */
    barrelConfig
        .encoder
        .positionConversionFactor(
            Constants.MechanismConstants.BarrelConstants.barrelEncoderPositionFactor)
        .velocityConversionFactor(
            Constants.MechanismConstants.BarrelConstants.barrelEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);

    /**
     * each sparkmax supports up to 4 slots for a preconfigured pid that it can then call using
     * SparkClosedLoopController defaults to slot 0 if not specified
     *
     * <p>feedbackSensor sets our sensor to the relative encoder
     *
     * <p>position wrapping makes it loop from the given range which we set to 0 and 2pi
     *
     * <p>then we set the pid configs ff = 0 because they do not take into account ks and their calc
     * isnt amazing instead we will implement ff as an arbff to be added later
     */
    barrelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, Math.PI * 2)

        /**
         * configure how often the motor receives/uses signals
         *
         * <p>set the velocity to always give output
         *
         * <p>set every periodic function in the motor to 20ms the standard
         */
        .pidf(
            Constants.MechanismConstants.BarrelConstants.barrelKp,
            0,
            Constants.MechanismConstants.BarrelConstants.barrelKd,
            0);

    /**
     * configure how often the motor receives/uses signals
     *
     * <p>set the velocity to always give output
     *
     * <p>set every periodic function in the motor to 20ms the standard
     */
    barrelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    /** configures the motor, retrying if faulting */
    makeItWork(
        barrelSpark,
        5,
        () ->
            barrelSpark.configure(
                barrelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    makeItWork(barrelSpark, 5, () -> barrelEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(BarrelIOInputs inputs) {

    // update barrel motor values, only accepting if no sticky fault present
    SparkUtil.stickyFault = false;
    ifOk(barrelSpark, barrelEncoder::getPosition, (value) -> inputs.barrelPositionRad = value);
    ifOk(
        barrelSpark, barrelEncoder::getVelocity, (value) -> inputs.barrelVelocityRadPerSec = value);
    ifOK(
        barrelSpark,
        new DoubleSupplier[] {barrelSpark::getAppliedOutput, barrelSpark::getBusVoltage},
        (values) -> inputs.barrelAppliedVolts = values[0] * values[1]);
    ifOk(barrelSpark, barrelSpark::getOutputCurrent, (value) -> inputs.barrelCurrentAmps = value);
    inputs.barrelConnected = barrelDebouncer.calculate(!SparkUtil.stickyFault);
  }

  /**
   * sets motor to specified open loop value
   *
   * @param output the volts to set
   */
  @Override
  public void setOpenLoop(double voltage) {
    barrelSpark.setVoltage(voltage);
  }

  /**
   * set motor to a desire angle, with an arbitrary feed forward to be calculated later
   *
   * @param rotation desire barrel angle in radians
   * @param arbFF the arbitrary ff unit in volts, to be added to the barrel pid output
   */
  @Override
  public void setBarrelPos(Rotation2d angle, double arbFF) {
    barrelController.setReference(
        angle.getRadians(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        arbFF,
        ArbFFUnits.kVoltage);
  }
}
