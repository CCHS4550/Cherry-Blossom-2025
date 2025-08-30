package frc.robot.Subsystems.Turret.Elevation;

import static frc.robot.Constants.MechanismConstants.ElevationConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** creates a elevation subsystem that can move according to manual input or lock to an angle */
public class Elevation extends SubsystemBase {
  private final ElevationIO
      io; // the interface used by elevation, will be defined as a ElevationIOSpark if real

  private final ElevationIOInputsAutoLogged inputs =
      new ElevationIOInputsAutoLogged(); // logged inputs of the elevation

  private final Alert elevationDCAlert;
  private final Alert elevationTwoDCAlert;

  // feed forward control
  private ArmFeedforward elevationFF =
      new ArmFeedforward(elevationFFKs, elevationFFKg, elevationFFKv, elevationFFKa);

  // motion profiling
  private TrapezoidProfile.Constraints constraints =
      new Constraints(
          elevationTrapezoidMaxVelo,
          elevationTrapezoidMaxAccel); // max speed and acceleration of the profile, in meters per
  // second and meters per second^2
  private TrapezoidProfile profile = new TrapezoidProfile(constraints); // create the profile
  private TrapezoidProfile.State state =
      new State(); // our current state, that will be updated by the profile into our desired
  // checkpoint on the way to the goal, which we will then get to
  private TrapezoidProfile.State goal = new State();

  public double manualControlVoltage =
      3; // set to different values before switching the state to manual
  public Rotation2d goalElevationRadians; // desired angle to go to in radians

  // what state we want the elevation to be
  public enum wantedElevationState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL, // when this is called it means the motor begins moving, so using manual is more like
    // turn this state on and off
    GOTO_ANGLE,
    IDLE
  }

  // what state the elevation is in
  private enum systemState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL, // when this is called it means the motor begins moving, so using manual is more like
    // turn this state on and off
    GOTO_ANGLE,
    IDLE
  }

  // initialize our states
  wantedElevationState WantedState = wantedElevationState.IDLE;
  systemState SystemState = systemState.IDLE;

  /**
   * constructor for the elevation
   *
   * @param io instance of ElevationIO or classes implementing BarrelIO
   */
  public Elevation(ElevationIO io) {
    this.io = io;

    elevationDCAlert = new Alert("primary elevation spark is disconnected", AlertType.kError);
    elevationTwoDCAlert = new Alert("secondary elevation spark is disconnected", AlertType.kError);
  }

  /**
   * will be called periodically
   *
   * <p>updating inputs and logging is NOT thread safe, so we use synchronized
   */
  @Override
  public void periodic() {
    // thread safe
    synchronized (inputs) {
      // update autologged inputs
      io.updateInputs(inputs);

      // stop the subsystem if disabled
      if (DriverStation.isDisabled()) {
        SystemState = systemState.IDLE;
        WantedState = wantedElevationState.IDLE;
        io.setElevationOpenLoop(0.0);
      }

      // set system state to match wanted state and deal with any changes that need to be made in
      // between states
      SystemState = handlStateTransitions();

      // turn the states into desired output
      applyStates();

      // update alerts
      elevationDCAlert.set(!inputs.elevationConnected);
      elevationTwoDCAlert.set(!inputs.elevationTwoConnected);
    }
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  public systemState handlStateTransitions() {
    if (SystemState != systemState.GOTO_ANGLE) {
      state =
          new State(
              inputs.elevationPositionRad,
              inputs.elevationVelocityRadPerSec); // if we are not currently tracking a trapezoidal
      // profile with the current state, set it to follow
      // the bots actual motion to be ready for beginning
      // the profile
    }
    return switch (WantedState) {
      case CHARACTERIZATION -> systemState.CHARACTERIZATION;
      case MANUAL -> systemState.MANUAL;
      case GOTO_ANGLE -> systemState.GOTO_ANGLE;
      case IDLE -> systemState.IDLE;
    };
  }
  // perform a desired outcome depending on our state
  public void applyStates() {
    switch (SystemState) {
      case CHARACTERIZATION:
        break;
      case MANUAL: // don't let the manual voltage apply if we are beyond the safe limit
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

  /**
   * @return if are angle is beyond the acceptable range or if we have hit the limit switch
   */
  public boolean beyondSafeLimit() {
    if (inputs.elevationPositionRad >= Math.PI / 2 // this might be too big idk
        || inputs.elevationPositionRad < 0
        || inputs.limitSwitchHit) {
      return true;
    }
    return false;
  }

  // calculate and go to our desired angle
  public void gotoAngle() {

    // if the desired angle is beyond what is safe, set it to our range and go there instead
    if (goalElevationRadians.getRadians() > Math.PI / 2) {
      goalElevationRadians = Rotation2d.kCCW_Pi_2; // this might be too big idk
    } else if (goalElevationRadians.getRadians() < 0) {
      goalElevationRadians = Rotation2d.kZero;
    }

    // if we hit the limit switch, move the elevation system back to safe range
    if (inputs.limitSwitchHit) {
      stopHittingLimit();
    } else {
      // actually go to the state if all checks are cleared

      // find our next checkpoint 0.02 seconds after our state
      state = profile.calculate(0.02, state, goal);

      // calculate the voltage necesary to meet the checkpoint's velocity
      double arbFF = elevationFF.calculate(state.position, state.velocity);

      // go to the checkpoint, while adding the feed forward value
      io.setElevationPos(Rotation2d.fromRadians(state.position), arbFF);
    }
  }

  // if we are at the limit switch, go back to safe range
  public void stopHittingLimit() {
    if (inputs.limitSwitchHit) {
      io.setElevationPos(Rotation2d.kCCW_Pi_2, 0.0); // this might be too big idk
    }
  }

  /**
   * set the desired angle to goto
   *
   * @param radians the wanted angle in radians
   */
  public void setGoalElevationRadians(double radians) {
    goalElevationRadians = Rotation2d.fromRadians(radians);
  }

  // just sets the goals of the trapezoid and subsystem
  public void beginTrapezoid(
      double
          radians) { // if we knew specifically what goal to go to we could set this in the class,
    // but because the state is goTo angle and not goto 60 degrees, we just set the
    // desired angle in superstructure or sontrol scheme
    goal = new State(radians, 0);
    setGoalElevationRadians(radians);
  }

  /**
   * sets the voltage used in manual
   *
   * @param volts the volts to use
   */
  public void setManualVoltage(double volts) {
    manualControlVoltage = volts;
  }

  /**
   * sets the elevation's wanted state should be the primary way of manipulating the elevation
   * outside of the class
   *
   * @param wanted the desired state
   */
  public void setWantedState(wantedElevationState wanted) {
    WantedState = wanted;
  }
}
