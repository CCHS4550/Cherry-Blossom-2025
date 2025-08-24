package frc.robot.Subsystems.Turret.Rotation;

import static frc.robot.Constants.MechanismConstants.RotationConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** creates a rotation than go spin freely or go to a robot oriented or field oriented angle */
public class Rotation extends SubsystemBase {

  private final RotationIO
      io; // the interface used by rotation, will bbe defined as a barrelIOSpark if real

  private final RotationIOInputsAutoLogged inputs =
      new RotationIOInputsAutoLogged(); // logged inputs of the rotation

  // rotation dc alert
  private final Alert rotationDCAlert;

  @SuppressWarnings("unused")
  private Rotation2d rotationRadiansFieldOriented = new Rotation2d();

  public double manualControlVoltage =
      3; // set to different values before switching the state to manual/test
  public Rotation2d
      wantedRotationRadiansBotOriented; // the angle of the barrel relative to the front of the
  // robot
  public Rotation2d wantedRotationRadiansFieldOriented; // angle of the barrel relative to the field
  public Pose2d targetLockPose; // the pose2d to aim at

  // Potential Bad Practice:
  // robot values, these may not be thread safe and also may lag behind real odometry values
  // implement a robotState class for better practice
  public Pose2d robotPose;
  public Rotation2d robotOrientation;
  public double robotYawVelo;

  // feed forward control
  private SimpleMotorFeedforward veloFF =
      new SimpleMotorFeedforward(rotationFFKs, rotationFFKv, rotationFFKa);

  // motion profiling
  private TrapezoidProfile.Constraints constraints =
      new Constraints(rotationTrapezoidMaxVelo, rotationTrapezoidMaxAccel);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);
  private TrapezoidProfile.State state = new State();
  private TrapezoidProfile.State goal = new State();

  // the desired rotation state
  public enum wantedRotationState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL,
    ROBOT_ORIENTED_ANGLE,
    BOT_ADJUSTED_ROBOT_ORIENTED_ANGLE,
    FIELD_ORIENTED_ANGLE,
    TARGET_LOCK,
    IDLE
  }
  // the current rotation state
  private enum systemState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    MANUAL,
    ROBOT_ORIENTED_ANGLE,
    BOT_ADJUSTED_ROBOT_ORIENTED_ANGLE,
    FIELD_ORIENTED_ANGLE,
    TARGET_LOCK,
    IDLE
  }

  // initialize states
  wantedRotationState WantedState = wantedRotationState.MANUAL;
  systemState SystemState = systemState.MANUAL;

  /**
   * constructor for the rotation
   *
   * @param io instance of RotationIO or classes implementing rotationIO
   */
  public Rotation(RotationIO io) {
    this.io = io;

    rotationDCAlert = new Alert("rotation motor is disconnected", AlertType.kError);
  }

  /**
   * will be called periodically
   *
   * <p>updating inputs and logging is NOT thread safe, so we use synchronized
   */
  @Override
  public void periodic() {
    synchronized (inputs) {
      // update the autologged inputs
      io.updateInputs(inputs);
      // update the turrets angle according to the field
      updateFieldOrientedAngle();

      // stop the subsystem if disabled
      if (DriverStation.isDisabled()) {
        setWantedState(
            wantedRotationState
                .IDLE); // this many set to 0 funcs is redundant but better safe than sorry
        SystemState = systemState.IDLE;
        setManualVoltage(0.0);
        io.setRotationOpenLoop(0.0);
      }

      // set system state to match wanted state and deal with any changes that need to be made in
      // between states
      SystemState = handleStateTransitions();

      // turn the states into desired output
      applyStates();

      // update alert
      rotationDCAlert.set(!inputs.rotationConnected);
    }
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  public systemState handleStateTransitions() {
    if (WantedState != wantedRotationState.ROBOT_ORIENTED_ANGLE
        || WantedState != wantedRotationState.BOT_ADJUSTED_ROBOT_ORIENTED_ANGLE) {
      state =
          new State(
              inputs.rotationPositionRad,
              inputs.rotationVelocityRadPerSec); // if we are not currently tracking a trapezoidal
      // profile with the current state, set it to follow
      // the bots actual motion to be ready for beginning
      // the profile
    }
    return switch (WantedState) {
      case CHARACTERIZATION -> systemState.CHARACTERIZATION;
      case MANUAL -> systemState.MANUAL;
      case ROBOT_ORIENTED_ANGLE -> systemState.ROBOT_ORIENTED_ANGLE;
      case BOT_ADJUSTED_ROBOT_ORIENTED_ANGLE -> systemState.BOT_ADJUSTED_ROBOT_ORIENTED_ANGLE;
      case FIELD_ORIENTED_ANGLE -> systemState.FIELD_ORIENTED_ANGLE;
      case TARGET_LOCK -> systemState.TARGET_LOCK;
      case IDLE -> systemState.IDLE;
    };
  }

  // perform a desired outcome depending on our state
  public void applyStates() {
    switch (SystemState) {
      case CHARACTERIZATION:
        break;
      case MANUAL: // will always move while manual is called, switch state to idle to stop movement
        io.setRotationOpenLoop(manualControlVoltage);
        break;
      case ROBOT_ORIENTED_ANGLE:
        wantedRotationRadiansBotOriented = optimizeAngle(wantedRotationRadiansBotOriented);
        io.setRotationPos(wantedRotationRadiansBotOriented);
        break;
      case BOT_ADJUSTED_ROBOT_ORIENTED_ANGLE:
        runRobotAdjustedAngle(); // does math to eliminate the bot rotation, then sets the motor
        // while accounting for bot rotational velocity
        break;
      case FIELD_ORIENTED_ANGLE:
        runFieldOrientatedAngle(); // converts to field orientated angle to robot orientated, then
        // calls robotAdjusted to finish
        break;
      case TARGET_LOCK:
        runFieldOrientatedAngle(); // finds angle to target, then calls runFieldOrientedAngle to
        // finish
        break;
      case IDLE:
        break;
    }
  }

  /** updates the turrets field orientated angle */
  public void updateFieldOrientedAngle() {
    double unwrappedRadians =
        (robotOrientation.plus(Rotation2d.fromRadians(inputs.rotationPositionRad)))
            .getRadians(); // findsd the angle by combining the robot relative rotation with the
    // bots rotation
    rotationRadiansFieldOriented =
        Rotation2d.fromRadians(wrapRadians(unwrappedRadians)); // wraps the value between 0 and 2pi
  }

  /** redundant we can kill this */
  public void runRobotAdjustedAngle() {
    Rotation2d adjustedAngle = wantedRotationRadiansBotOriented.minus(robotOrientation);
    adjustedAngle = optimizeAngle(adjustedAngle);
    goal =
        new State(wantedRotationRadiansBotOriented.getRadians() - robotOrientation.getRadians(), 0);
    goal = new State(optimizeAngle(Rotation2d.fromRadians(goal.position)).getRadians(), 0);
    state = profile.calculate(0.02, state, goal);
    double arbFF = veloFF.calculate(state.velocity - robotYawVelo);
    io.setRotationPos(adjustedAngle, arbFF);
  }

  /**
   * sets the optomizes our wanted angle, then creates a motion profile and feedforward to go to it
   */
  public void runNonAdjustedAngle() {
    // set the angle to the lowest value, ie. 270 -> -90
    Rotation2d optomizedAngle = optimizeAngle(wantedRotationRadiansBotOriented);
    goal = new State(optomizedAngle.getRadians(), 0.0);

    // find our next checkpoint 0.02 seconds after our state
    state = profile.calculate(0.02, state, goal);

    // calculate the voltage necesary to meet the checkpoint's velocity
    double arbFF = veloFF.calculate(state.velocity);

    // go to the checkpoint, while adding the feed forward value
    io.setRotationPos(Rotation2d.fromRadians(state.position), arbFF);
  }

  /**
   * sets the optomizes our wanted angle, then creates a motion profile and feedforward to go to it
   *
   * @param veloCompensation if the final velocity given to feed forward should compensate for the
   *     robots yaw velocity
   */
  public void runNonAdjustedAngle(boolean veloCompensation) {
    // set the angle to the lowest value, ie. 270 -> -90
    Rotation2d optomizedAngle = optimizeAngle(wantedRotationRadiansBotOriented);

    // set the goal to be optomized
    goal = new State(optomizedAngle.getRadians(), 0.0);

    // find our next checkpoint 0.02 seconds after our state
    state = profile.calculate(0.02, state, goal);

    // calculate the voltage necesary to meet the checkpoint's velocity
    double arbFF = 0.0;
    if (veloCompensation) {
      arbFF = veloFF.calculate(state.velocity - this.robotYawVelo);
    } else {
      arbFF = veloFF.calculate(state.velocity);
    }

    // go to the checkpoint, while adding the feed forward value
    io.setRotationPos(Rotation2d.fromRadians(state.position), arbFF);
  }

  /**
   * turns the field oriented angle into a robot oriented one, wrapping to avoid going beyond 0 and
   * 2pi, then call runNonAdjustedAngle to set rotation to it
   */
  public void runFieldOrientatedAngle() {

    // convert angle
    wantedRotationRadiansBotOriented =
        Rotation2d.fromRadians(
            wrapRadians((wantedRotationRadiansFieldOriented.minus(robotOrientation)).getRadians()));
    // goal = new State(optimizeAngle(wantedRotationRadiansBotOriented).getRadians(), 0.0);
    // shouldn't be necesary

    // goTo the angle while compensating for bot movement
    runNonAdjustedAngle(true);
  }

  /**
   * finds the field oriented angle to the target position and sets it as goal, then calls
   * runFieldOrientedAngle to handle all logic from theere
   */
  public void runTargetLock() {
    double targetFieldAngle =
        Math.atan2(
            targetLockPose.getY() - robotPose.getY(), targetLockPose.getX() - robotPose.getX());
    wantedRotationRadiansFieldOriented = Rotation2d.fromRadians(targetFieldAngle);
    runFieldOrientatedAngle();
  }

  /**
   * sets the rotation's wanted state should be the primary way of manipulating the rotation outside
   * of the class
   *
   * @param wantedState the desired state
   */
  public void setWantedState(wantedRotationState wantedRotation) {
    WantedState = wantedRotation;
  }

  /**
   * use sets what the rotation should do in the manual control state
   *
   * @param volts volts to set to
   */
  public void setManualVoltage(double volts) {
    manualControlVoltage = volts;
  }

  /**
   * Minimize the rotation required to get to a point If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desired the rotation we wish to optomize
   * @return a rotation 2d set to be as close as possible to the other while maintaining the
   *     previous rotational status
   */
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

  /** wraps an angle between 0 and 2pi radians */
  public double wrapRadians(double radians) {
    return (radians % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
  }

  /** sets robot information again, potential bad practice */
  public void setRobotAngle(Rotation2d rotation) {
    robotOrientation = rotation;
  }

  /** sets robot information again, potential bad practice */
  public void setRobotVelo(double velo) {
    robotYawVelo = velo;
  }

  /** sets robot information again, potential bad practice */
  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public void setGoal(
      double
          radians) { // if we knew specifically what goal to go to we could set this in the class,
    // but because the state is goTo angle and not goto 60 degrees, we just set the
    // desired angle in superstructure or sontrol scheme
    goal = new State(optimizeAngle(Rotation2d.fromRadians(radians)).getRadians(), 0);
    wantedRotationRadiansBotOriented = Rotation2d.fromRadians(radians);
  }

  public void setGoalFieldOriented(double radians) {
    // if we knew specifically what goal to go to we could set this in the class,
    // but because the state is goTo angle and not goto 60 degrees, we just set the
    // desired angle in superstructure or sontrol scheme
    wantedRotationRadiansFieldOriented = Rotation2d.fromRadians(radians);
  }

  /**
   * used for initiating our turrets field orientated rotation by setting it as the same as our bot
   * + any pre determined starting turn
   */
  public void setrotationRadiansFieldOriented(Rotation2d adjustment) {
    rotationRadiansFieldOriented = robotOrientation.plus(adjustment);
  }
}
