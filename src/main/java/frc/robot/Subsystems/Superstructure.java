package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Turret.Barrels.Barrel;
import frc.robot.Subsystems.Turret.Barrels.Barrel.wantedBarrelState;
import frc.robot.Subsystems.Turret.Elevation.Elevation;
import frc.robot.Subsystems.Turret.Elevation.Elevation.wantedElevationState;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics.wantedPneumaticsState;
import frc.robot.Subsystems.Turret.Rotation.Rotation;
import frc.robot.Subsystems.Turret.Rotation.Rotation.wantedRotationState;
import org.littletonrobotics.junction.Logger;

/** creates a periodic state machine used for determining the behavior of the entire robot */
public class Superstructure extends SubsystemBase {
  // potential bad practice
  public boolean
      isRunningCommand; // exists in order to prevent the periodic state machine from calling the
  // same command
  // multiple times

  // the subsystems used in the super structure
  public Pneumatics pneumatics;
  public Barrel barrels;
  Elevation elevation;
  Rotation rotation;
  Drive drive;

  // the desired state for our bot to be in
  public enum wantedState {
    ELEVATION_OPENLOOP_UP,
    ELEVATION_OPENLOOP_DOWN,
    ROTATION_OPENLOOP_CLOCKWISE,
    ROTATION_OPENLOOP_COUNTERCLOCKWISE,
    BARREL_TEST,
    ROTATE_60_DEGREES_BOT_ORIENTED,
    FILLING_AIR,
    SHOOT_ONE,
    SHOOT_ALL,
    IDLE
  }

  // the state our bot is in
  private enum systemState {
    ELEVATION_OPENLOOP_UP,
    ELEVATION_OPENLOOP_DOWN,
    ROTATION_OPENLOOP_CLOCKWISE,
    ROTATION_OPENLOOP_COUNTERCLOCKWISE,
    BARREL_TEST,
    ROTATE_60_DEGREES_BOT_ORIENTED,
    FILLING_AIR,
    SHOOT_ONE,
    SHOOT_ALL,
    IDLE
  }

  // initialize states
  public wantedState WantedState = wantedState.IDLE;
  systemState SystemState = systemState.IDLE;

  /**
   * Constructor for the super structure
   *
   * @param pneumatics instance of PneumaticsIO or classes implementing PneumaticsIO
   * @param barrels instance of BarrelIO or classes implementing BarrelIO
   * @param elevation instance of ElevationIO or classes implementing ElevationIO
   * @param rotation instance of RotationIO or classes implementing RotationIO
   * @param drive the drive train
   */
  public Superstructure(
      Pneumatics pneumatics, Barrel barrels, Elevation elevation, Rotation rotation, Drive drive) {
    this.pneumatics = pneumatics;
    this.barrels = barrels;
    this.elevation = elevation;
    this.rotation = rotation;
    this.drive = drive;

    // sets the turrets field oriented rotation to be the same as the bots, if not, change the
    // adjustment rotation 2d that get added on the bots
  }

  /**
   * runs periodically
   *
   * <p>thread safe is not needed because already dealt with in subsystems
   *
   * <p>again sending drive information this way is bad practice, should use a robotState class
   */
  @Override
  public void periodic() {

    // stop if disabled
    if (DriverStation.isDisabled()) {
      setWantedState(wantedState.IDLE);
      SystemState = systemState.IDLE;
    }

    // set system state to match wanted state and deal with any changes that need to be made in
    // between states
    SystemState = handleStateTransitions();

    // turn the states into desired output
    applyStates();
    Logger.recordOutput("Subsystems/Superstructure/WantedState", WantedState);
    Logger.recordOutput("Subsystems/Superstructure/SystemState", SystemState);
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  public systemState handleStateTransitions() {
    return switch (WantedState) {
      case ELEVATION_OPENLOOP_UP -> systemState.ELEVATION_OPENLOOP_UP;
      case ELEVATION_OPENLOOP_DOWN -> systemState.ELEVATION_OPENLOOP_DOWN;
      case ROTATION_OPENLOOP_CLOCKWISE -> systemState.ROTATION_OPENLOOP_CLOCKWISE;
      case ROTATION_OPENLOOP_COUNTERCLOCKWISE -> systemState.ROTATION_OPENLOOP_COUNTERCLOCKWISE;
      case ROTATE_60_DEGREES_BOT_ORIENTED -> systemState.ROTATE_60_DEGREES_BOT_ORIENTED;
      case BARREL_TEST -> systemState.BARREL_TEST;
      case FILLING_AIR -> systemState.FILLING_AIR;
      case SHOOT_ONE -> systemState.SHOOT_ONE;
      case SHOOT_ALL -> systemState.SHOOT_ALL;
      case IDLE -> systemState.IDLE;
    };
  }

  // perform a desired outcome depending on our state
  public void applyStates() {
    switch (SystemState) {
      case ELEVATION_OPENLOOP_UP:
        elevation.setManualVoltage(3);
        elevation.setWantedState(wantedElevationState.MANUAL);
        break;
      case ELEVATION_OPENLOOP_DOWN:
        elevation.setManualVoltage(-3);
        elevation.setWantedState(wantedElevationState.MANUAL);
        break;
      case ROTATION_OPENLOOP_CLOCKWISE:
        rotation.setManualVoltage(3);
        rotation.setWantedState(wantedRotationState.MANUAL);
        break;
      case ROTATION_OPENLOOP_COUNTERCLOCKWISE:
        rotation.setManualVoltage(-3);
        rotation.setWantedState(wantedRotationState.MANUAL);
        break;
      case BARREL_TEST:
        barrels.setWantedState(wantedBarrelState.TEST);
        System.out.println("superstructure called");
        break;
      case ROTATE_60_DEGREES_BOT_ORIENTED:
        rotation.setGoal(Math.PI / 3);
        rotation.setWantedState(wantedRotationState.ROBOT_ORIENTED_ANGLE);
        break;
      case FILLING_AIR:
        pneumatics.setWantedState(wantedPneumaticsState.FILLING_AIR_TANK);
        break;
      case SHOOT_ONE:
        break;
      case SHOOT_ALL:
        break;
      case IDLE:
        elevation.setWantedState(wantedElevationState.IDLE);
        rotation.setWantedState(wantedRotationState.IDLE);
        pneumatics.setWantedState(wantedPneumaticsState.IDLE);
        barrels.setWantedState(wantedBarrelState.IDLE);
        break;
    }
  }

  /**
   * sets the bots's wanted state should be the primary way of manipulating the superstructure
   * outside of the class
   *
   * @param wantedState the desired state
   */
  public void setWantedState(wantedState WantedState) {
    this.WantedState = WantedState;
  }
}
