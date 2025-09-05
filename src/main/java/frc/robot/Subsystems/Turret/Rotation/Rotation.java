public class Rotation extends SubsystemBase{
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

  public void setRotationVoltage (double voltage){
    rotationSpark.setVoltage(voltage);
  }

  public void setRotationPos (Rotation2d angle){
    rotationController.setReference(angle.getRadians(), SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public Command setVoltage (double voltage){
    return new RunCommand(()-> setRotationVoltage(voltage);)
  }

  public Command setPosition (Rotation2d angle){
    return new RunCommand(() - > setRotationPos(angle));
  }

  
}
