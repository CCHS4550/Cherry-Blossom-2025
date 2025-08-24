package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Turret.Barrels.Barrel;
import frc.robot.Subsystems.Turret.Barrels.Barrel.wantedBarrelState;
import frc.robot.Subsystems.Turret.Elevation.Elevation;
import frc.robot.Subsystems.Turret.Elevation.Elevation.wantedElevationState;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics.wantedPneumaticsState;
import frc.robot.Subsystems.Turret.Rotation.Rotation;
import frc.robot.Subsystems.Turret.Rotation.Rotation.wantedRotationState;

/**
 * creates a periodic state machine used for determining the behavior of the entire robot
 */
public class Superstructure extends SubsystemBase {
  
  // potential bad practice
  public boolean isRunningCommand; // exists in order to prevent the periodic state machine from calling the same command
  // multiple times

  // the subsystems used in the super structure
  Pneumatics pneumatics;
  Barrel barrels;
  Elevation elevation;
  Rotation rotation;
  Drive drive;

  // the desired state for our bot to be in
  public enum wantedState {
    ELEVATION_OPENLOOP_UP,
    ELEVATION_OPENLOOP_DOWN,
    ROTATION_OPENLOOP_CLOCKWISE,
    ROTATION_OPENLOOP_COUNTERCLOCKWISE,
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
    FILLING_AIR,
    SHOOT_ONE,
    SHOOT_ALL,
    IDLE
  }

  // initialize states
  wantedState WantedState = wantedState.IDLE;
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
    
    //sets the turrets field oriented rotation to be the same as the bots, if not, change the adjustment rotation 2d that get added on the bots
    sendDriveInfoToRotation();
    rotation.setrotationRadiansFieldOriented(Rotation2d.kZero);
  }

  /** 
   * runs periodically
   * 
   * thread safe is not needed because already dealt with in subsystems
   * 
   * again sending drive information this way is bad practice, should use a robotState class
   */
  @Override
  public void periodic() {
    // sending drive information this way is bad practice, should use a robotState class
    sendDriveInfoToRotation();
    
    // stop if disabled
    if (DriverStation.isDisabled()) {
      setWantedState(wantedState.IDLE);
      SystemState = systemState.IDLE;
    }
    
    //set system state to match wanted state and deal with any changes that need to be made in between states
    SystemState = handleStateTransitions();
    
    // turn the states into desired output
    applyStates();
  }
  
  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more complex judgements on what state to goto if so desired
   * 
   * @return the systemstate that our systemState variable will be set to
   */
  public systemState handleStateTransitions() {
    return switch (WantedState) {
      case ELEVATION_OPENLOOP_UP -> systemState.ELEVATION_OPENLOOP_UP;
      case ELEVATION_OPENLOOP_DOWN -> systemState.ELEVATION_OPENLOOP_DOWN;
      case ROTATION_OPENLOOP_CLOCKWISE -> systemState.ROTATION_OPENLOOP_CLOCKWISE;
      case ROTATION_OPENLOOP_COUNTERCLOCKWISE -> systemState.ROTATION_OPENLOOP_COUNTERCLOCKWISE;
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
        rotation.setManualVoltage(0);
        rotation.setWantedState(wantedRotationState.MANUAL);
      case FILLING_AIR:
        pneumatics.setWantedState(wantedPneumaticsState.FILLING_AIR_TANK);
      case SHOOT_ONE:
        if (!isRunningCommand) {
          shootThenIndex();
        } else {
          WantedState = wantedState.IDLE;
        }
        break;
      case SHOOT_ALL:
        if (!isRunningCommand) {
          shootSix();
        } else {
          WantedState = wantedState.IDLE;
        }
        break;
      case IDLE:
        elevation.setWantedState(wantedElevationState.IDLE);
        rotation.setWantedState(wantedRotationState.IDLE);
        pneumatics.setWantedState(wantedPneumaticsState.IDLE);
        barrels.setWantedState(wantedBarrelState.IDLE);
    }
  }

/**
 * shoots and indexes all 6 barrels of the T-shirt cannon, then ends and sets the state back to idle
 * 
 * sets and resets the isRunningCommand boolean automatically
 * 
 * should end on a redundant index but that shouldnt do any harm
 * 
 * @return the command to shoot all 6 and then reset state and boolean
 */
  public Command shootSix() {
    isRunningCommand = true; // set isRunningCommand true to not overwhelm the scheduler
    return new SequentialCommandGroup(
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        new InstantCommand(() -> WantedState = wantedState.IDLE), // got back to idle so this state can be called again
        new InstantCommand(() -> isRunningCommand = false)); // reset the boolean
  }

  /**
 * shoots then indexes 1 barrel of the T-shirt cannon, then ends and sets the state back to idle if we just want to shoot 1
 * 
 * sets and resets the isRunningCommand boolean automatically if we just want to shoot 1
 * 
 * @return the command to shoot then index and to reset or not
 */
  public Command shootThenIndex() {
    isRunningCommand = true; // set isRunningCommand true to not overwhelm the scheduler
    return new SequentialCommandGroup(
        new InstantCommand(() -> pneumatics.setWantedState(wantedPneumaticsState.SHOOT)), // shoot using the pneumatics state
        new WaitUntilCommand(() -> !pneumatics.isRunningCommand), // wait until that sequence has finishied
        new InstantCommand(() -> barrels.setWantedState(wantedBarrelState.INDEX)), // index the barrel
        new WaitUntilCommand(() -> barrels.isAtAngle), // wait until we are at angle, then move on
        
        // reset state if not in the SHOOT_ALL state, otherwise, return a null command
        new ConditionalCommand(
            new InstantCommand(() -> WantedState = wantedState.IDLE),
            new InstantCommand(),
            () -> WantedState != wantedState.SHOOT_ALL),

        // reset boolean if not in the SHOOT_ALL state, otherwise return a null command
        new ConditionalCommand(
            new InstantCommand(() -> isRunningCommand = false),
            new InstantCommand(),
            () -> WantedState != wantedState.SHOOT_ALL));
  }

  // sends odometry information to the rotation to use for field orientated
  // again, potential bad practice
  public void sendDriveInfoToRotation() {
    rotation.setRobotPose(drive.getPose());
    rotation.setRobotAngle(drive.getRotation());
    rotation.setRobotVelo(drive.getYawVelocity());
  }

  /**
   * sets the bots's wanted state 
   * should be the primary way of manipulating the superstructure outside of the class
   * 
   * @param wantedState the desired state
   */
  public void setWantedState(wantedState WantedState) {
    this.WantedState = WantedState;
  }
}
