package frc.robot.Subsystems.Turret.Rotation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotation extends SubsystemBase {
  private final RotationIO io;

  private final RotationIOInputsAutoLogged inputs = new RotationIOInputsAutoLogged();

  public double manualControlVoltage = 3;
  public Rotation2d rotationRadiansBotOriented;
  public Rotation2d robotOrientation;
  public double robotYawVelo;

  private SimpleMotorFeedforward veloFF = new SimpleMotorFeedforward(0, 0, 0);
  private TrapezoidProfile.Constraints constraints = new Constraints(0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State state = new State();
  private TrapezoidProfile.State goal = new State();

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
    io.updateInputs(inputs);

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
    if (WantedState != wantedState.ROBOT_ORIENTED_ANGLE
        || WantedState != wantedState.FIELD_ORIENTED_ANGLE) {
      state = new State(inputs.rotationPositionRad, inputs.rotationVelocityRadPerSec);
    }
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
    goal = new State(rotationRadiansBotOriented.getRadians() - robotOrientation.getRadians(), 0);
    goal = new State(optimizeAngle(Rotation2d.fromRadians(goal.position)).getRadians(), 0);
    state = profile.calculate(0.02, state, goal);
    double arbFF = veloFF.calculate(state.velocity - robotYawVelo);
    io.setRotationPos(adjustedAngle, arbFF);
  }

  public void runNonAdjustedAngle() {
    Rotation2d optomizedAngle = optimizeAngle(rotationRadiansBotOriented);
    state = profile.calculate(0.02, state, goal);
    double arbFF = veloFF.calculate(state.velocity);
    io.setRotationPos(optomizedAngle, arbFF);
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

  public void beginTrapezoid(double radians) { // This must be called seperately from periodic
    goal = new State(optimizeAngle(Rotation2d.fromRadians(radians)).getRadians(), 0);
    rotationRadiansBotOriented = Rotation2d.fromRadians(radians);
  }
}
