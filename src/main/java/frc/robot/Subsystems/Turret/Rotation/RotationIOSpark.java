package frc.robot.Subsystems.Turret.Rotation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class RotationIOSpark implements RotationIO{
    private final SparkBase rotationSpark;
    private final RelativeEncoder rotationEncoder;
    private final SparkClosedLoopController rotationController;
    private final Debouncer driveDebouncer = new Debouncer(0.5);

    public RotationIOSpark(){
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
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            
    }

}
