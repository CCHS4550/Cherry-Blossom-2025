package frc.robot.Subsystems.Turret.Elevation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevation extends SubsystemBase {
  private final ElevationIO io;

  private final ElevationIOInputsAutoLogged inputs = new ElevationIOInputsAutoLogged();

  private ArmFeedforward elevationFF = new ArmFeedforward(0, 0, 0);
  private TrapezoidProfile.Constraints constraints = new Constraints(0, 0);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State state = new State();
  private TrapezoidProfile.State goal = new State();

  public double manualControlVoltage = 3;
  public Rotation2d goalElevationRadians;

  public enum wantedState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL,
    GOTO_ANGLE,
    IDLE
  }

  private enum systemState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL,
    GOTO_ANGLE,
    IDLE
  }

  wantedState WantedState = wantedState.MANUAL;
  systemState SystemState = systemState.MANUAL;

  public Elevation(ElevationIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      SystemState = systemState.IDLE;
      WantedState = wantedState.IDLE;
      io.setElevationOpenLoop(0.0);
    }

    handlStateTransitions();
    applyStates();
  }

  public systemState handlStateTransitions() {
    if (SystemState != systemState.GOTO_ANGLE) {
      state = new State(inputs.elevationPositionRad, inputs.elevationVelocityRadPerSec);
    }
    return switch (WantedState) {
      case CHARACTERIZATION -> systemState.CHARACTERIZATION;
      case MANUAL -> systemState.MANUAL;
      case GOTO_ANGLE -> systemState.GOTO_ANGLE;
      case IDLE -> systemState.IDLE;
    };
  }

  public void applyStates() {
    switch (SystemState) {
      case CHARACTERIZATION:
        break;
      case MANUAL:
        if (beyondSafeLimit()) {
          stopHittingLimit();
          break;
        } else {
          io.setElevationOpenLoop(manualControlVoltage);
          break;
        }
      case GOTO_ANGLE:
        gotoAngle();
        break;
      case IDLE:
        break;
    }
  }

  public boolean beyondSafeLimit() {
    if (inputs.elevationPositionRad >= Math.PI / 2
        || inputs.elevationPositionRad < 0
        || inputs.limitSwitchHit) {
      return true;
    }
    return false;
  }

  public void gotoAngle() {
    if (goalElevationRadians.getRadians() > Math.PI / 2) {
      goalElevationRadians = Rotation2d.kCCW_Pi_2;
    } else if (goalElevationRadians.getRadians() < 0) {
      goalElevationRadians = Rotation2d.kZero;
    }
    if (inputs.limitSwitchHit) {
      stopHittingLimit();
    } else {
      state = profile.calculate(0.02, state, goal);
      double arbFF = elevationFF.calculate(state.position, state.velocity);
      io.setElevationPos(goalElevationRadians, arbFF);
    }
  }

  public void stopHittingLimit() {
    if (inputs.limitSwitchHit) {
      io.setElevationPos(Rotation2d.kCCW_Pi_2, 0.0);
    }
  }

  public void setGoalElevationRadians(double radians) {
    goalElevationRadians = Rotation2d.fromRadians(radians);
  }

  public void beginTrapezoid(double radians) { // This must be called seperately from periodic
    goal = new State(radians, 0);
    setGoalElevationRadians(radians);
  }
}
