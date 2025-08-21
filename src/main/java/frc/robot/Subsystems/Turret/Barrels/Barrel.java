package frc.robot.Subsystems.Turret.Barrels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Barrel extends SubsystemBase {
  private final BarrelIO io;

  private final BarrelIOInputsAutoLogged inputs = new BarrelIOInputsAutoLogged();

  public double manualControlVoltage = 3;
  private Rotation2d barrelAngle = Rotation2d.kZero;
  public boolean isAtAngle = true;

  private SimpleMotorFeedforward veloFF = new SimpleMotorFeedforward(Constants.MechanismConstants.BarrelConstants.barrelFFKs, Constants.MechanismConstants.BarrelConstants.barrelFFKv, Constants.MechanismConstants.BarrelConstants.barrelFFKa);
  private TrapezoidProfile.Constraints constraints = new Constraints(0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State state = new State();
  private TrapezoidProfile.State goal = new State();

  public enum wantedBarrelState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    TEST,
    INDEX,
    IDLE
  }

  private enum systemState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    TEST,
    INDEX,
    IDLE
  }

  wantedBarrelState WantedState = wantedBarrelState.IDLE;
  systemState SystemState = systemState.IDLE;

  public Barrel(BarrelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      setWantedState(
          wantedBarrelState
              .IDLE); // this many set to 0 funcs is redundant but better safe than sorry
      SystemState = systemState.IDLE;
      io.setOpenLoop(0.0);
    }
    SystemState = handleStateTransitions();
    applyStates();
    isAtAngle();
  }

  public systemState handleStateTransitions() {
    if (WantedState != wantedBarrelState.INDEX) {
      state = new State(inputs.barrelPositionRad, inputs.barrelVelocityRadPerSec);
    }
    return switch (WantedState) {
      case CHARACTERIZATION -> systemState.CHARACTERIZATION;
      case TEST -> systemState.TEST;
      case INDEX -> systemState.INDEX;
      case IDLE -> systemState.IDLE;
    };
  }

  public void applyStates() {
    switch (SystemState) {
      case CHARACTERIZATION:
        break;
      case TEST:
        io.setOpenLoop(manualControlVoltage);
        break;
      case INDEX:
        indexBarrel();
        break;
      case IDLE:
        break;
    }
  }

  public void indexBarrel() {
    state = profile.calculate(0.02, state, goal);
    double arbFF = veloFF.calculate(state.velocity);
    io.setBarrelPos(Rotation2d.fromRadians(state.position), arbFF);
  }

  public void nextAngle() {
    Rotation2d angle = barrelAngle.plus(Rotation2d.fromRadians(Math.PI / 3));
    if (angle.getRadians() >= 360 || angle.getRadians() <= 0) {
      angle = Rotation2d.kZero;
    }
    goal = new State(angle.getRadians(), 0);
    barrelAngle = angle;
  }

  public void setWantedState(wantedBarrelState wantedBarrel) {
    WantedState = wantedBarrel;
    if (wantedBarrel == wantedBarrelState.INDEX) {
      isAtAngle = false;
      nextAngle();
    }
  }

  public boolean isAtAngle() {
    if (MathUtil.isNear(
        inputs.barrelPositionRad, barrelAngle.getRadians(), Units.degreesToRadians(0.05))) {
      isAtAngle = true;
      setWantedState(wantedBarrelState.IDLE);
    }
    return isAtAngle;
  }
}
