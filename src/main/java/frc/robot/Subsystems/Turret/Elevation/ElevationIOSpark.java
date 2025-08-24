package frc.robot.Subsystems.Turret.Elevation;

import static frc.robot.Util.SparkUtil.ifOK;
import static frc.robot.Util.SparkUtil.ifOk;
import static frc.robot.Util.SparkUtil.makeItWork;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Util.SparkUtil;
import java.util.function.DoubleSupplier;

// define a class that uses the interface BarrelIO
// used to initiate the hardware used on the row and define the interface methods
public class ElevationIOSpark implements ElevationIO {
  
  // the limit switch
  private final DigitalInput limitSwitch;
  
  // the motors and encoders of the elevation subsystem
  private final SparkBase elevationSpark;
  private final SparkBase elevationSparkTwo;
  private final RelativeEncoder elevationEncoder;
  private final RelativeEncoder elevationEncoderTwo;
  
  //closed loop control, but only one because the other motor will simply follow
  private final SparkClosedLoopController elevationController;
  
  //checks if the sparks are disconnected
  private final Debouncer elevationDebouncer = new Debouncer(0.5);
  private final Debouncer elevationDebouncerTwo = new Debouncer(0.5);

  /**
   * constructor for the elevation subsystem
   */
  public ElevationIOSpark() {
    
    // fully define the elevation motors
    elevationSpark =
        new SparkMax(Constants.MechanismConstants.elevationCanID, MotorType.kBrushless);
    elevationSparkTwo =
        new SparkMax(Constants.MechanismConstants.elevationCanIDTwo, MotorType.kBrushless);
    
    // fully define the limit switch
    limitSwitch = new DigitalInput(Constants.MechanismConstants.elevationLimitSwitchID);
    
    // fully define the encoders
    elevationEncoder = elevationSpark.getEncoder();
    elevationEncoderTwo = elevationSparkTwo.getEncoder();
    
    // declare the elevation's closed loop control
    elevationController = elevationSpark.getClosedLoopController();

    //config for the primary motor of the elevation subsystem, this is the one that will do all the pid calcs and be called for information
    var elevationConfig = new SparkMaxConfig();
    
    // if it should be inverted
    elevationConfig.inverted(Constants.MechanismConstants.elevationInverted);
    
    /**
     * idleMode is Brake, stay at position when stopped
     * set the smart current limit to avoid going over what the motor can handle 
     * 
     * <p> voltage compensation = 12 because working with 12v car battery
     */
    elevationConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.elevationCurrentLimit)
        .voltageCompensation(12.0);
    
    /**
     * configures the encoder 
     * 
     * <p> position factor converts rotations to radians while accounting
     * for any gearing 
     * 
     * <p>velocity factor converts rotations/min to radians/sec while accounting for
     * any gearing
     *
     * <p>this is now automatically applied anytime we request motor information
     *
     * <p>average depth is the bit size of the sampling depth, must be a power of 2
     */
    elevationConfig
        .encoder
        .positionConversionFactor(Constants.MechanismConstants.elevationEncoderPositionFactor)
        .velocityConversionFactor(Constants.MechanismConstants.elevationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);

    /**
     * each sparkmax supports up to 4 slots for a preconfigured pid that it can then call using
     * SparkClosedLoopController defaults to slot 0 if not specified
     *
     * <p>feedbackSensor sets our sensor to the relative encoder
     * 
     * no position wrapping because if the elevation goes 360 degrees something is very wrong
     *
     * <p>then we set the pid configs ff = 0 because they do not take into account ks and their calc
     * isnt amazing instead we will implement ff as an arbff to be added later
     */
    elevationConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(
            Constants.MechanismConstants.elevationKp,
            0,
            Constants.MechanismConstants.elevationKd,
            0);

    /**
     * configure how often the motor receives/uses signals
     *
     * <p>set the velocity to always give output
     *
     * <p>set every periodic function in the motor to 20ms the standard
     */
    elevationConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    /** configures the motor, retrying if faulting */
    makeItWork(
        elevationSpark,
        5,
        () ->
            elevationSpark.configure(
                elevationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    /** set motor position to 0, retrying if faulting */
    makeItWork(elevationSpark, 5, () -> elevationEncoder.setPosition(0.0));

    /**
     * configuration for the secondary elevation motor
     */
    var elevationTwoConfig = new SparkMaxConfig();
    
    // if it should be inverted
    elevationTwoConfig.inverted(Constants.MechanismConstants.elevationTwoInverted);
    
    // do whatever the primary motor does, so no fine control of this motor needed
    elevationTwoConfig.follow(elevationSpark);

    /**
     * idleMode is Brake, stay at position when stopped
     * set the smart current limit to avoid going over what the motor can handle 
     * 
     * <p> voltage compensation = 12 because working with 12v car battery
     */
    elevationTwoConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.elevationTwoCurrentLimit)
        .voltageCompensation(12.0);

    /**
     * configure how often the motor receives/uses signals
     *
     * <p>set the velocity to always give output
     *
     * <p>set every periodic function in the motor to 20ms the standard
     */
    elevationTwoConfig
        .encoder
        .positionConversionFactor(Constants.MechanismConstants.elevationEncoderPositionFactor)
        .velocityConversionFactor(Constants.MechanismConstants.elevationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);

    /**
     * configure how often the motor receives/uses signals
     *
     * <p>set the velocity to always give output
     *
     * <p>set every periodic function in the motor to 20ms the standard
     */
    elevationTwoConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    /** configures the motor, retrying if faulting */
    makeItWork(
        elevationSparkTwo,
        5,
        () ->
            elevationSparkTwo.configure(
                elevationTwoConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    /** sets encoder to 0, retrying if faulting */
    makeItWork(elevationSparkTwo, 5, () -> elevationEncoderTwo.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevationIOInputs inputs) {
    
    // update primary motor values, only accepting if no sticky fault present
    SparkUtil.stickyFault = false;
    ifOk(
        elevationSpark,
        elevationEncoder::getPosition,
        (value) -> inputs.elevationPositionRad = value);
    ifOk(
        elevationSpark,
        elevationEncoder::getVelocity,
        (value) -> inputs.elevationVelocityRadPerSec = value);
    ifOK(
        elevationSpark,
        new DoubleSupplier[] {elevationSpark::getAppliedOutput, elevationSpark::getBusVoltage},
        (values) -> inputs.elevationAppliedVolts = values[0] * values[1]);
    ifOk(
        elevationSpark,
        elevationSpark::getOutputCurrent,
        (value) -> inputs.elevationCurrentAmps = value);
    inputs.elevationConnected = elevationDebouncer.calculate(!SparkUtil.stickyFault);

    // update secondary motor values, only accepting if no sticky fault present
    SparkUtil.stickyFault = false;
    ifOk(
        elevationSparkTwo,
        elevationEncoderTwo::getPosition,
        (value) -> inputs.elevationTwoPositionRad = value);
    ifOk(
        elevationSparkTwo,
        elevationEncoderTwo::getVelocity,
        (value) -> inputs.elevationTwoVelocityRadPerSec = value);
    ifOK(
        elevationSparkTwo,
        new DoubleSupplier[] {
          elevationSparkTwo::getAppliedOutput, elevationSparkTwo::getBusVoltage
        },
        (values) -> inputs.elevationTwoAppliedVolts = values[0] * values[1]);
    ifOk(
        elevationSparkTwo,
        elevationSparkTwo::getOutputCurrent,
        (value) -> inputs.elevationTwoCurrentAmps = value);
    inputs.elevationTwoConnected = elevationDebouncerTwo.calculate(!SparkUtil.stickyFault);

    // update the limitswitch's status
    inputs.limitSwitchHit = limitSwitch.get();
  }

  /**
   * sets motor to specified open loop value
   * 
   * <p> the secondary motor follows the primary one, so only need to set the primary motor
   *
   * @param output the volts to set
   */
  @Override
  public void setElevationOpenLoop(double voltage) {
    elevationSpark.setVoltage(voltage);
  }
  
  /**
   * set turn motor to a desire angle, with an arbitrary feed forward to be calculated later
   * 
   * <p> the secondary motor follows the primary one, so only need to set the primary motor
   *
   * @param rotation desire module angle in radians
   * 
   * @param arbFF the arbitrary ff unit in volts, to be added to the elevation pid output
   */

  @Override
  public void setElevationPos(Rotation2d angle, double arbFF) {
    elevationController.setReference(
        angle.getRadians(),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        arbFF,
        ArbFFUnits.kVoltage);
  }
}
