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

public class BarrelIOSpark implements BarrelIO {
  private final SparkBase barrelSpark;
  private final RelativeEncoder barrelEncoder;
  private final SparkClosedLoopController barrelController;
  private final Debouncer barrelDebouncer = new Debouncer(0.5);

  public BarrelIOSpark() {
    barrelSpark = new SparkMax(Constants.MechanismConstants.barrelCanID, MotorType.kBrushless);
    barrelEncoder = barrelSpark.getEncoder();
    barrelController = barrelSpark.getClosedLoopController();

    var barrelConfig = new SparkMaxConfig();
    barrelConfig.inverted(Constants.MechanismConstants.barrelInverted);
    barrelConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.barrelCurrentLimit)
        .voltageCompensation(12.0);
    barrelConfig
        .encoder
        .positionConversionFactor(Constants.MechanismConstants.barrelEncoderPositionFactor)
        .velocityConversionFactor(Constants.MechanismConstants.barrelEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);
    barrelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, Math.PI * 2)
        .pidf(Constants.MechanismConstants.barrelKp, 0, Constants.MechanismConstants.barrelKd, 0);
    barrelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
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

  @Override
  public void setOpenLoop(double voltage) {
    barrelSpark.setVoltage(voltage);
  }

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
