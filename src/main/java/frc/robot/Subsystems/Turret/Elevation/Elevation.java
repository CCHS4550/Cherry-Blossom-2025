package frc.robot.Subsystems.Turret.Elevation;

import static frc.robot.Constants.MechanismConstants.ElevationConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevation extends SubsystemBase {
  // the limit switch
  // commented out for testing purposes
  private final DigitalInput limitSwitch;

  // the motors and encoders of the elevation subsystem
  private final SparkBase elevationSpark;
  private final SparkBase elevationSparkTwo;
  private final RelativeEncoder elevationEncoder;
  private final RelativeEncoder elevationEncoderTwo;

  // closed loop control, but only one because the other motor will simply follow
  private final SparkClosedLoopController elevationController;

  //elevation PID
  private PIDController elevationPID;

  // checks if the sparks are disconnected
  private final Debouncer elevationDebouncer = new Debouncer(0.5);
  private final Debouncer elevationDebouncerTwo = new Debouncer(0.5);

  /** constructor for the elevation subsystem */
  public ElevationIOSpark() {

    // config elevation PID
    elevationPID = new PIDController (0.3, 0,0);

    // fully define the elevation motors
    elevationSpark =
        new SparkMax(
            Constants.MechanismConstants.ElevationConstants.elevationCanID, MotorType.kBrushless);
    elevationSparkTwo =
        new SparkMax(
            Constants.MechanismConstants.ElevationConstants.elevationCanIDTwo,
            MotorType.kBrushless);
    // fully define the limit switch
    limitSwitch =
        new DigitalInput(Constants.MechanismConstants.ElevationConstants.elevationLimitSwitchID);
    // fully define the encoders
    elevationEncoder = elevationSpark.getEncoder();
    elevationEncoderTwo = elevationSparkTwo.getEncoder();

    // declare the elevation's closed loop control
    elevationController = elevationSpark.getClosedLoopController();

    // config for the primary motor of the elevation subsystem, this is the one that will do all the
    // pid calcs and be called for information
    var elevationConfig = new SparkMaxConfig();

    // if it should be inverted
    elevationConfig.inverted(Constants.MechanismConstants.ElevationConstants.elevationInverted);

    /**
     * idleMode is Brake, stay at position when stopped set the smart current limit to avoid going
     * over what the motor can handle
     *
     * <p>voltage compensation = 12 because working with 12v car battery
     */
    elevationConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.ElevationConstants.elevationCurrentLimit)
        .voltageCompensation(12.0);

    /**
     * configures the encoder
     *
     * <p>position factor converts rotations to radians while accounting for any gearing
     *
     * <p>velocity factor converts rotations/min to radians/sec while accounting for any gearing
     *
     * <p>this is now automatically applied anytime we request motor information
     *
     * <p>average depth is the bit size of the sampling depth, must be a power of 2
     */
    elevationConfig
        .encoder
        .positionConversionFactor(
            Constants.MechanismConstants.ElevationConstants.elevationEncoderPositionFactor)
        .velocityConversionFactor(
            Constants.MechanismConstants.ElevationConstants.elevationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);

    /**
     * each sparkmax supports up to 4 slots for a preconfigured pid that it can then call using
     * SparkClosedLoopController defaults to slot 0 if not specified
     *
     * <p>feedbackSensor sets our sensor to the relative encoder
     *
     * <p>no position wrapping because if the elevation goes 360 degrees something is very wrong
     *
     * <p>then we set the pid configs ff = 0 because they do not take into account ks and their calc
     * isnt amazing instead we will implement ff as an arbff to be added later
     */
    elevationConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(
            Constants.MechanismConstants.ElevationConstants.elevationKp,
            0,
            Constants.MechanismConstants.ElevationConstants.elevationKd,
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

    /** configuration for the secondary elevation motor */
    var elevationTwoConfig = new SparkMaxConfig();

    // if it should be inverted
    elevationTwoConfig.inverted(
        Constants.MechanismConstants.ElevationConstants.elevationTwoInverted);

    // do whatever the primary motor does, so no fine control of this motor needed
    elevationTwoConfig.follow(elevationSpark);

    /**
     * idleMode is Brake, stay at position when stopped set the smart current limit to avoid going
     * over what the motor can handle
     *
     * <p>voltage compensation = 12 because working with 12v car battery
     */
    elevationTwoConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.ElevationConstants.elevationTwoCurrentLimit)
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
        .positionConversionFactor(
            Constants.MechanismConstants.ElevationConstants.elevationEncoderPositionFactor)
        .velocityConversionFactor(
            Constants.MechanismConstants.ElevationConstants.elevationEncoderVeloFactor)
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
    // makeItWork(
    //     elevationSparkTwo,
    //     5,
    //     () ->
    //         elevationSparkTwo.configure(
    //             elevationTwoConfig,
    //             ResetMode.kResetSafeParameters,
    //             PersistMode.kPersistParameters));
  }

  public void setElevationVoltage (double voltage){
    elevationSpark.setVoltage(voltage);
  }

  public void setElevation (double power){
    elevationSpark.set(power);
  }

  public Command setVoltage (double voltage){
    return new RunCommand (()-> setElevationVoltage(voltage));
  }

  public Command goToPosition (Rotation2d angle){
    return new RunCommand (()-> setElevationPos (angle));
  }

  public void goToAngle (Rotation2d angle){
    setElevation (elevationPID.calculate(angle.toRadians()); // TODO - angle to encoder value conversion
  }

  public void setElevationPos (Rotation2d angle){
    elevationController.setReference(angle.getRadians(), SparkBase.ControlType.kMAXMotionPositionControl);
  }
}
