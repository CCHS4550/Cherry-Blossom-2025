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

public class ElevationIOSpark implements ElevationIO {
  private final DigitalInput limitSwitch;
  private final SparkBase elevationSpark;
  private final SparkBase elevationSparkTwo;
  private final RelativeEncoder elevationEncoder;
  private final RelativeEncoder elevationEncoderTwo;
  private final SparkClosedLoopController elevationController;
  private final Debouncer elevationDebouncer = new Debouncer(0.5);
  private final Debouncer elevationDebouncerTwo = new Debouncer(0.5);

  public ElevationIOSpark() {
    elevationSpark =
        new SparkMax(Constants.MechanismConstants.ElevationConstants.elevationCanID, MotorType.kBrushless);
    elevationSparkTwo =
        new SparkMax(Constants.MechanismConstants.ElevationConstants.elevationCanIDTwo, MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.MechanismConstants.ElevationConstants.elevationLimitSwitchID);
    elevationEncoder = elevationSpark.getEncoder();
    elevationEncoderTwo = elevationSparkTwo.getEncoder();
    elevationController = elevationSpark.getClosedLoopController();

    var elevationConfig = new SparkMaxConfig();
    elevationConfig.inverted(Constants.MechanismConstants.ElevationConstants.elevationInverted);
    elevationConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.ElevationConstants.elevationCurrentLimit)
        .voltageCompensation(12.0);
    elevationConfig
        .encoder
        .positionConversionFactor(Constants.MechanismConstants.ElevationConstants.elevationEncoderPositionFactor)
        .velocityConversionFactor(Constants.MechanismConstants.ElevationConstants.elevationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);
    elevationConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false)
        .pidf(
            Constants.MechanismConstants.ElevationConstants.elevationKp,
            0,
            Constants.MechanismConstants.ElevationConstants.elevationKd,
            0);
    elevationConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    makeItWork(
        elevationSpark,
        5,
        () ->
            elevationSpark.configure(
                elevationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    makeItWork(elevationSpark, 5, () -> elevationEncoder.setPosition(0.0));

    var elevationTwoConfig = new SparkMaxConfig();
    elevationTwoConfig.inverted(Constants.MechanismConstants.ElevationConstants.elevationTwoInverted);
    elevationTwoConfig.follow(elevationSpark);
    elevationTwoConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.ElevationConstants.elevationTwoCurrentLimit)
        .voltageCompensation(12.0);
    elevationTwoConfig
        .encoder
        .positionConversionFactor(Constants.MechanismConstants.ElevationConstants.elevationEncoderPositionFactor)
        .velocityConversionFactor(Constants.MechanismConstants.ElevationConstants.elevationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);
    elevationTwoConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    makeItWork(
        elevationSparkTwo,
        5,
        () ->
            elevationSparkTwo.configure(
                elevationTwoConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    makeItWork(elevationSparkTwo, 5, () -> elevationEncoderTwo.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevationIOInputs inputs) {
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

    inputs.limitSwitchHit = limitSwitch.get();
  }

  @Override
  public void setElevationOpenLoop(double voltage) {
    elevationSpark.setVoltage(voltage);
  }

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
