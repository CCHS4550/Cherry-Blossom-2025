package frc.robot.Subsystems.Turret.Barrels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * creates an barrel that can spin freely or go to a desired angle
 */
public class Barrel extends SubsystemBase {
  private final BarrelIO io; // the interface used by barrel, will be defined as barrelIOSpark if real

  private final BarrelIOInputsAutoLogged inputs = new BarrelIOInputsAutoLogged(); // logged inputs of the barrel

  // alert for motor disconnection
  private final Alert barrelDCAlert;
  
  public double manualControlVoltage = 3; //set to different values before switching the state to manual/test
  private Rotation2d barrelAngle = Rotation2d.kZero; // desired rotation to index the barrel to
  public boolean isAtAngle = true; //tracks if the barrel is at the desired index angle, so the external command that calls index knows when to move on

  // feed forward control
  private SimpleMotorFeedforward veloFF = new SimpleMotorFeedforward(0, 0, 0); //feedforward to calculate voltage needed to get to the given setpoint of the trapezoidal motion profile
  
  // motion profiling
  private TrapezoidProfile.Constraints constraints = new Constraints(0, 0); // max speed and acceleration of the profile, in meters per second and meters per second^2
  private TrapezoidProfile profile = new TrapezoidProfile(constraints); // create the profile
  private TrapezoidProfile.State state = new State(); // our current state, that will be updated by the profile into our desired checkpoint on the way to the goal, which we will then get to
  private TrapezoidProfile.State goal = new State(); // our desired state

  // state we want the barrel to be in
  public enum wantedBarrelState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    TEST,
    INDEX,
    IDLE
  }

  // state the barrel is in
  private enum systemState {
    CHARACTERIZATION, // not going to code this part b/c pid constants already found and there's
    // like a million sources on how to tune pid
    TEST,
    INDEX,
    IDLE
  }

  // initialize our states
  wantedBarrelState WantedState = wantedBarrelState.IDLE;
  systemState SystemState = systemState.IDLE;

  /**
   * constructor for the barrel
   * 
   * @param io instance of BarrelIO or classes implementing BarrelIO
   */
  public Barrel(BarrelIO io) {
    this.io = io;

    barrelDCAlert = new Alert("Disconnected barrel spark", AlertType.kError);
  }

  /**
   * will be called periodically
   * 
   * updating inputs and logging is NOT thread safe, so we use synchronized
   */
  @Override
  public void periodic() {
    // thread safe
    synchronized(inputs){
      //update autologged inputs
      io.updateInputs(inputs);

      //stop the subsystem if disabled
      if (DriverStation.isDisabled()) {
      setWantedState(
          wantedBarrelState
              .IDLE); // this many set to 0 funcs is redundant but better safe than sorry
      SystemState = systemState.IDLE;
      io.setOpenLoop(0.0);
    }
    
    //set system state to match wanted state and deal with any changes that need to be made in between states
    SystemState = handleStateTransitions();
    
    // turn the states into desired output
    applyStates();
    
    // if we are at the angle, switch state to IDLE, this only works on barrel because there is very little force of gravity
    // this should be done this way on barrel because it needs to calculate the new rotation angle everytime the wanted state switches back to index
    // this could likely be solved better by implementing a HOLD_AT_STATE state that still calls pid to go to a state, but doesn't update the state like index
    // TODO: add a HOLD_AT_ANGLE state
    isAtAngle(); //boolean is unimportant in the subsystem, but is called in external commands to know when to move on
    
    // update alert
    barrelDCAlert.set(!inputs.barrelConnected);
    }
  }
  
  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more complex judgements on what state to goto if so desired
   * 
   * @return the systemstate that our systemState variable will be set to
   */
  public systemState handleStateTransitions() {
    if (WantedState != wantedBarrelState.INDEX) {
      state = new State(inputs.barrelPositionRad, inputs.barrelVelocityRadPerSec); // if we are not currently tracking a trapezoidal profile with the current state, set it to follow the bots actual motion to be ready for beginning the profile
    }
    return switch (WantedState) {
      case CHARACTERIZATION -> systemState.CHARACTERIZATION;
      case TEST -> systemState.TEST;
      case INDEX -> systemState.INDEX;
      case IDLE -> systemState.IDLE;
    };
  }
  // perform a desired outcome depending on our state
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
    // find our next checkpoint 0.02 seconds after our state
    state = profile.calculate(0.02, state, goal);
    
    // calculate the voltage necesary to meet the checkpoint's velocity
    double arbFF = veloFF.calculate(state.velocity);
    
    // go to the checkpoint, while adding the feed forward value
    io.setBarrelPos(Rotation2d.fromRadians(state.position), arbFF);
  }

  // next barrel to index to, wrapping to 0 if making a full turn
  public void nextAngle() {
    
    // add 60  degrees
    Rotation2d angle = barrelAngle.plus(Rotation2d.fromRadians(Math.PI / 3));
    
    // wrap the angle if needed
    if (angle.getRadians() >= 360 || angle.getRadians() <= 0) {
      angle = Rotation2d.kZero;
    }
    
    // set the trapezoids goal and the pids goal to the new angle
    goal = new State(angle.getRadians(), 0);
    barrelAngle = angle;
  }
  /**
   * sets the barrel's wanted state 
   * should be the primary way of manipulating the barrel outside of the class
   * 
   * @param wantedState the desired state
   */
  public void setWantedState(wantedBarrelState wantedBarrel) {
    WantedState = wantedBarrel;
    if (wantedBarrel == wantedBarrelState.INDEX) {
      isAtAngle = false;
      nextAngle();
    }
  }

  /**
   * If the barrel is at angle, set the state to idle so the next time wanted state is INDEX, we find the next angle
   * 
   * @return if the barrel is at angle, this boolean exists mainly for external command timing
   */
  public boolean isAtAngle() {
    if (MathUtil.isNear(
        inputs.barrelPositionRad, barrelAngle.getRadians(), Units.degreesToRadians(0.05))) {
      isAtAngle = true;
      setWantedState(wantedBarrelState.IDLE);
    }
    return isAtAngle;
  }
}
