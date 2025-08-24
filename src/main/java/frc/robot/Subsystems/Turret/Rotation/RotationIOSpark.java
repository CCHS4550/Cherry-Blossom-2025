package frc.robot.Subsystems.Turret.Rotation;

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

// define a class that uses the interface RotationIO
// used to initiate the hardware used on the row and define the interface methods
public class RotationIOSpark implements RotationIO {
  
  // the motor and encoder
  private final SparkBase rotationSpark;
  private final RelativeEncoder rotationEncoder;
  
  // closed loop control for the rotation
  private final SparkClosedLoopController rotationController;
  
  // check if the rotation motor is disconnected
  private final Debouncer rotationDebouncer = new Debouncer(0.5);

  /**
   * constructor for rotation
   */
  public RotationIOSpark() {


    // fully define rotation spark and encoder  
    rotationSpark =
        new SparkMax(
            Constants.MechanismConstants.RotationConstants.rotationCanID, MotorType.kBrushless);
    rotationEncoder = rotationSpark.getEncoder();
    
    // declare the encoders closed loop control
    rotationController = rotationSpark.getClosedLoopController();

    // config for the rotation motor
    var rotationConfig = new SparkMaxConfig();


    // if it is inverted
    rotationConfig.inverted(Constants.MechanismConstants.RotationConstants.rotationInverted);
    
    /**
     * idleMode is Brake, stay at position when stopped
     * set the smart current limit to avoid going over what the motor can handle voltage compensation = 12 because
     * working with 12v car battery
     */
    rotationConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.RotationConstants.rotationCurrentLimit)
        .voltageCompensation(12.0);

    /**
     * configures the encoder position factor converts rotations to radians while accounting
     * for any gearing 
     * 
     * <p>velocity factor converts rotations/min to radians/sec while accounting for
     * any gearing
     *
     * <p>this is now automatically applied anytime we request motor information
     *
     * <p>average depth is the bit size of the sampling depth, must be a power of 2
     */
    rotationConfig
        .encoder
        .positionConversionFactor(
            Constants.MechanismConstants.RotationConstants.rotationEncoderPositionFactor)
        .velocityConversionFactor(
            Constants.MechanismConstants.RotationConstants.rotationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);

    /**
     * each sparkmax supports up to 4 slots for a preconfigured pid that it can then call using
     * SparkClosedLoopController defaults to slot 0 if not specified
     *
     * <p>feedbackSensor sets our sensor to the relative encoder 
     * 
     * position wrapping makes it loop
     * from the given range which we set to 0 and 2pi
     *
     * <p>then we set the pid configs ff = 0 because they do not take into account ks and their calc
     * isnt amazing instead we will implement ff as an arbff to be added later
     */
    rotationConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, Math.PI * 2)
        .pidf(
            Constants.MechanismConstants.RotationConstants.rotationKp,
            0,
            Constants.MechanismConstants.RotationConstants.rotationKd,
            0);

    /**
     * configure how often the motor receives/uses signals
     *
     * <p>set the velocity to always give output
     *
     * <p>set every periodic function in the motor to 20ms the standard
     */  
    rotationConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    /** configures the motor, retrying if faulting */
    makeItWork(
        rotationSpark,
        5,
        () ->
            rotationSpark.configure(
                rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    /** sets the encoder to 0, retrying if faulting */
    makeItWork(rotationSpark, 5, () -> rotationEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(RotationIOInputs inputs) {
    
    //update rotation motor values, only accepting if no stick fault present
    SparkUtil.stickyFault = false;
    ifOk(
        rotationSpark, rotationEncoder::getPosition, (value) -> inputs.rotationPositionRad = value);
    ifOk(
        rotationSpark,
        rotationEncoder::getVelocity,
        (value) -> inputs.rotationVelocityRadPerSec = value);
    ifOK(
        rotationSpark,
        new DoubleSupplier[] {rotationSpark::getAppliedOutput, rotationSpark::getBusVoltage},
        (values) -> inputs.rotationAppliedVolts = values[0] * values[1]);
    ifOk(
        rotationSpark,
        rotationSpark::getOutputCurrent,
        (value) -> inputs.rotationCurrentAmps = value);
    inputs.rotationConnected = rotationDebouncer.calculate(!SparkUtil.stickyFault);
  }

  /**
   * sets motor to specified open loop value
   *
   * @param output the volts to set
   */
  @Override
  public void setRotationOpenLoop(double voltage) {
    rotationSpark.setVoltage(voltage);
  }

  /**
   * set turn motor to a desire angle, without feed forward
   *
   * @param rotation desire rotation angle in radians
   */
  @Override
  public void setRotationPos(Rotation2d angle) {
    rotationController.setReference(
        angle.getRadians(), SparkBase.ControlType.kMAXMotionPositionControl);
  }

  /**
   * set motor to a desire angle, with an arbitrary feed forward to be calculated later
   *
   * @param rotation desire rotation angle in radians
   * 
   * @param arbFF the arbitrary ff unit in volts, to be added to the barrel pid output
   */
  @Override
  public void setRotationPos(Rotation2d angle, double arbFF) {
    rotationController.setReference(
        angle.getRadians(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        arbFF,
        ArbFFUnits.kVoltage);
  }
}
