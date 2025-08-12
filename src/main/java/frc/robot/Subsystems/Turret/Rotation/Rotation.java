package frc.robot.Subsystems.Turret.Rotation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotation extends SubsystemBase {
  private final RotationIO io;

  private final RotationIOInputsAutoLogged inputs = new RotationIOInputsAutoLogged();

  private SimpleMotorFeedforward veloCompensationFF = new SimpleMotorFeedforward(0, 0, 0);

  public double manualControlVoltage = 3;
  public Rotation2d rotationRadiansBotOriented;
  public Rotation2d robotOrientation;
  public double robotYawVelo;

  public enum wantedState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL,
    ROBOT_ORIENTED_ANGLE,
    FIELD_ORIENTED_ANGLE,
    IDLE
  }

  private enum systemState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL,
    ROBOT_ORIENTED_ANGLE,
    FIELD_ORIENTED_ANGLE,
    IDLE
  }

  wantedState WantedState = wantedState.MANUAL;
  systemState SystemState = systemState.MANUAL;

  public Rotation(RotationIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setWantedState(
          wantedState.IDLE); // this many set to 0 funcs is redundant but better safe than sorry
      SystemState = systemState.IDLE;
      setManualVoltage(0.0);
      io.setRotationOpenLoop(0.0);
    }
    SystemState = handleStateTransitions();
    applyStates();
  }

  public systemState handleStateTransitions() {
    return switch (WantedState) {
      case CHARACTERIZATION -> systemState.CHARACTERIZATION;
      case MANUAL -> systemState.MANUAL;
      case ROBOT_ORIENTED_ANGLE -> systemState.ROBOT_ORIENTED_ANGLE;
      case FIELD_ORIENTED_ANGLE -> systemState.FIELD_ORIENTED_ANGLE;
      case IDLE -> systemState.IDLE;
    };
  }

  public void applyStates() {
    switch (SystemState) {
      case CHARACTERIZATION:
        break;
      case MANUAL: // will always move while manual is called, switch state to idle to stop movement
        io.setRotationOpenLoop(manualControlVoltage);
        break;
      case ROBOT_ORIENTED_ANGLE:
        rotationRadiansBotOriented = optimizeAngle(rotationRadiansBotOriented);
        io.setRotationPos(rotationRadiansBotOriented);
        break;
      case FIELD_ORIENTED_ANGLE:
        runRobotAdjustedAngle();
        break;
      case IDLE:
        break;
    }
  }

  public void runRobotAdjustedAngle() {
    Rotation2d adjustedAngle = rotationRadiansBotOriented.minus(robotOrientation);
    adjustedAngle = optimizeAngle(adjustedAngle);
    double arbFF = veloCompensationFF.calculate(-robotYawVelo);
    io.setRotationPos(adjustedAngle, arbFF);
  }

  public void setWantedState(wantedState wanted) {
    WantedState = wanted;
  }

  public void setManualVoltage(double volts) {
    manualControlVoltage = volts;
  }

  public Rotation2d optimizeAngle(
      Rotation2d
          desired) { // math for this taken from the serve state implementation prewritten by frc
    Rotation2d currentAngle = Rotation2d.fromRadians(inputs.rotationPositionRad);
    var delta = desired.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      desired.times(-1);
      desired = desired.rotateBy(Rotation2d.kPi);
    }
    return desired;
  }

  public void setRobotAngle(Rotation2d rotation) {
    robotOrientation = rotation;
  }

  public void setRobotVelo(double velo) {
    robotYawVelo = velo;
  }
}
