package frc.robot.Subsystems.Turret.Rotation;

import static frc.robot.Constants.MechanismConstants.RotationConstants.*;

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

  private SimpleMotorFeedforward veloFF =
      new SimpleMotorFeedforward(rotationFFKs, rotationFFKv, rotationFFKa);
  private TrapezoidProfile.Constraints constraints =
      new Constraints(rotationTrapezoidMaxVelo, rotationTrapezoidMaxAccel);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State state = new State();
  private TrapezoidProfile.State goal = new State();

  public enum wantedRotationState {
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

  wantedRotationState WantedState = wantedRotationState.MANUAL;
  systemState SystemState = systemState.MANUAL;

  public Rotation(RotationIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    synchronized (inputs) {
      io.updateInputs(inputs);

      if (DriverStation.isDisabled()) {
        setWantedState(
            wantedRotationState
                .IDLE); // this many set to 0 funcs is redundant but better safe than sorry
        SystemState = systemState.IDLE;
        setManualVoltage(0.0);
        io.setRotationOpenLoop(0.0);
      }
      SystemState = handleStateTransitions();
      applyStates();
    }
  }

  public systemState handleStateTransitions() {
    if (WantedState != wantedRotationState.ROBOT_ORIENTED_ANGLE
        || WantedState != wantedRotationState.FIELD_ORIENTED_ANGLE) {
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
        runNonAdjustedAngle();
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
    goal = new State(adjustedAngle.getRadians(), 0.0);
    state = profile.calculate(0.02, state, goal);
    double arbFF = veloFF.calculate(state.velocity - robotYawVelo);
    io.setRotationPos(Rotation2d.fromRadians(state.position), arbFF);
  }

  public void runNonAdjustedAngle() {
    Rotation2d optomizedAngle = optimizeAngle(rotationRadiansBotOriented);
    goal = new State(optomizedAngle.getRadians(), 0.0);
    state = profile.calculate(0.02, state, goal);
    double arbFF = veloFF.calculate(state.velocity);
    io.setRotationPos(Rotation2d.fromRadians(state.position), arbFF);
  }

  public void setWantedState(wantedRotationState wantedRotation) {
    WantedState = wantedRotation;
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

  public void setGoal(
      double
          radians) { // if we knew specifically what goal to go to we could set this in the class,
    // but because the state is goTo angle and not goto 60 degrees, we just set the
    // desired angle in superstructure or sontrol scheme
    goal = new State(optimizeAngle(Rotation2d.fromRadians(radians)).getRadians(), 0);
    rotationRadiansBotOriented = Rotation2d.fromRadians(radians);
  }
}
