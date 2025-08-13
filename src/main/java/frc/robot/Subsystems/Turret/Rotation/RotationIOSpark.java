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

public class RotationIOSpark implements RotationIO {
  private final SparkBase rotationSpark;
  private final RelativeEncoder rotationEncoder;
  private final SparkClosedLoopController rotationController;
  private final Debouncer rotationDebouncer = new Debouncer(0.5);

  public RotationIOSpark() {
    rotationSpark = new SparkMax(Constants.MechanismConstants.rotationCanID, MotorType.kBrushless);
    rotationEncoder = rotationSpark.getEncoder();
    rotationController = rotationSpark.getClosedLoopController();

    var rotationConfig = new SparkMaxConfig();
    rotationConfig.inverted(Constants.MechanismConstants.rotationInverted);
    rotationConfig.encoder.inverted(Constants.MechanismConstants.rotationEncoderInverted);
    rotationConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MechanismConstants.rotationCurrentLimit)
        .voltageCompensation(12.0);
    rotationConfig
        .encoder
        .positionConversionFactor(Constants.MechanismConstants.rotationEncoderPositionFactor)
        .velocityConversionFactor(Constants.MechanismConstants.rotationEncoderVeloFactor)
        .uvwMeasurementPeriod(20)
        .uvwAverageDepth(2);
    rotationConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, Math.PI * 2)
        .pidf(
            Constants.MechanismConstants.rotationKp, 0, Constants.MechanismConstants.rotationKd, 0);
    rotationConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    makeItWork(
        rotationSpark,
        5,
        () ->
            rotationSpark.configure(
                rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    makeItWork(rotationSpark, 5, () -> rotationEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(RotationIOInputs inputs) {
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

  @Override
  public void setRotationOpenLoop(double voltage) {
    rotationSpark.setVoltage(voltage);
  }

  @Override
  public void setRotationPos(Rotation2d angle) {
    rotationController.setReference(
        angle.getRadians(), SparkBase.ControlType.kMAXMotionPositionControl);
  }

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
