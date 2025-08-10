package frc.robot.Subsystems.Drive.Module;

import static frc.robot.Util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.SparkOdometryThread;
import frc.robot.Util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d rotationOffset;

  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final Queue<Double> timeQueue;
  private final Queue<Double> drivePosQueue;
  private final Queue<Double> turnPosQueue;

  private final Debouncer driveDebouncer = new Debouncer(0.5);
  private final Debouncer turnDebouncer = new Debouncer(0.5);

  public ModuleIOSpark(int module) {
    rotationOffset =
        switch (module) {
          case 0 -> Constants.DriveConstants.frontLeftOffset;
          case 1 -> Constants.DriveConstants.frontRightOffset;
          case 2 -> Constants.DriveConstants.backLeftOffset;
          case 3 -> Constants.DriveConstants.backRightOffset;
          default -> new Rotation2d();
        };
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftTurnCanId;
              case 1 -> Constants.DriveConstants.frontRightTurnCanId;
              case 2 -> Constants.DriveConstants.backLeftTurnCanId;
              case 3 -> Constants.DriveConstants.backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Constants.DriveConstants.frontLeftDriveCanId;
              case 1 -> Constants.DriveConstants.frontRightDriveCanId;
              case 2 -> Constants.DriveConstants.backLeftDriveCanId;
              case 3 -> Constants.DriveConstants.backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder(); // don't need an absolute encoder for drive
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // turn motor config
    var turnConfig = new SparkMaxConfig();
    switch (module) {
      case 0 -> turnConfig.inverted(Constants.DriveConstants.frontLeftTurnInverted);
      case 1 -> turnConfig.inverted(Constants.DriveConstants.frontRightTurnInverted);
      case 2 -> turnConfig.inverted(Constants.DriveConstants.backLeftTurnInverted);
      case 3 -> turnConfig.inverted(Constants.DriveConstants.backRightTurnInverted);
      default -> turnConfig.inverted(false);
    }
    switch (module) {
      case 0 -> turnConfig.absoluteEncoder.inverted(
          Constants.DriveConstants.frontLeftTurnEncoderInverted);
      case 1 -> turnConfig.absoluteEncoder.inverted(
          Constants.DriveConstants.frontRightTurnEncoderInverted);
      case 2 -> turnConfig.absoluteEncoder.inverted(
          Constants.DriveConstants.backLeftTurnEncoderInverted);
      case 3 -> turnConfig.absoluteEncoder.inverted(
          Constants.DriveConstants.backRightTurnEncoderInverted);
    }
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12);
    turnConfig
        .absoluteEncoder
        .positionConversionFactor(Constants.DriveConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            Constants.DriveConstants.turnPIDMinInput, Constants.DriveConstants.turnPIDMaxInput)
        .pidf(Constants.DriveConstants.turnKp, 0.0, Constants.DriveConstants.turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(
            (int) (1000.0 / Constants.DriveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.makeItWork(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    var driveConfig = new SparkMaxConfig();
    switch (module) {
      case 0 -> driveConfig.inverted(Constants.DriveConstants.frontLeftDriveInverted);
      case 1 -> driveConfig.inverted(Constants.DriveConstants.frontRightDriveInverted);
      case 2 -> driveConfig.inverted(Constants.DriveConstants.backLeftDriveInverted);
      case 3 -> driveConfig.inverted(Constants.DriveConstants.backRightDriveInverted);
      default -> driveConfig.inverted(false);
    }
    switch (module) {
      case 0 -> driveConfig.encoder.inverted(
          Constants.DriveConstants.frontLeftDriveEncoderInverted);
      case 1 -> driveConfig.encoder.inverted(
          Constants.DriveConstants.frontRightDriveEncoderInverted);
      case 2 -> driveConfig.encoder.inverted(Constants.DriveConstants.backLeftDriveEncoderInverted);
      case 3 -> driveConfig.encoder.inverted(
          Constants.DriveConstants.backRightDriveEncoderInverted);
    }
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DriveConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(Constants.DriveConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(Constants.DriveConstants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.DriveConstants.driveKp, 0.0, Constants.DriveConstants.driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    makeItWork(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.makeItWork(
        driveSpark,
        5,
        () -> driveEncoder.setPosition(0.0)); // reset the sparkmax encoder b/c its not absolute

    // make the queues
    timeQueue = SparkOdometryThread.getInstance().makeTimeQueue();
    drivePosQueue =
        SparkOdometryThread.getInstance()
            .registerSparkSignal(driveSpark, driveEncoder::getPosition);
    turnPosQueue =
        SparkOdometryThread.getInstance().registerSparkSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // drive
    SparkUtil.stickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOK(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveDebouncer.calculate(!SparkUtil.stickyFault);

    // turn
    SparkUtil.stickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(rotationOffset));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOK(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnDebouncer.calculate(!SparkUtil.stickyFault);

    // odometry
    inputs.odometryTimestamps = timeQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePosQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPosQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(rotationOffset))
            .toArray(Rotation2d[]::new);
    timeQueue.clear();
    drivePosQueue.clear();
    turnPosQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelo(double velo) { // radians per second
    double ffvolts =
        Constants.DriveConstants.driveKs * Math.signum(velo)
            + Constants.DriveConstants.driveKv * velo;
    driveController.setReference(
        velo, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffvolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPos(Rotation2d rotation) {
    double setPoint =
        MathUtil.inputModulus(
            rotation.plus(rotationOffset).getRadians(),
            Constants.DriveConstants.turnPIDMinInput,
            Constants.DriveConstants.turnPIDMaxInput);
    turnController.setReference(setPoint, ControlType.kPosition);
  }
}
