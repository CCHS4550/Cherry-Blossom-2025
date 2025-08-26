package frc.robot.Subsystems;

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

public class Superstructure extends SubsystemBase {
  private boolean isRunningCommand;

  Pneumatics pneumatics;
  Barrel barrels;
  Elevation elevation;
  Rotation rotation;
  Drive drive;

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

  wantedState WantedState = wantedState.IDLE;
  systemState SystemState = systemState.IDLE;

  public Superstructure(
      Pneumatics pneumatics, Barrel barrels, Elevation elevation, Rotation rotation, Drive drive) {
    this.pneumatics = pneumatics;
    this.barrels = barrels;
    this.elevation = elevation;
    this.rotation = rotation;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    sendDriveInfoToRotation();
    if (DriverStation.isDisabled()) {
      setWantedState(wantedState.IDLE);
      SystemState = systemState.IDLE;
    }
    SystemState = handleStateTransitions();
    applyStates();
  }

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

  public Command shootSix() {
    isRunningCommand = true;
    return new SequentialCommandGroup(
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        shootThenIndex(),
        new InstantCommand(() -> WantedState = wantedState.IDLE),
        new InstantCommand(() -> isRunningCommand = false));
  }

  public Command shootThenIndex() {
    isRunningCommand = true;
    return new SequentialCommandGroup(
        new InstantCommand(() -> pneumatics.setWantedState(wantedPneumaticsState.SHOOT)),
        new WaitUntilCommand(() -> !pneumatics.isRunningCommand),
        new InstantCommand(() -> barrels.setWantedState(wantedBarrelState.INDEX)),
        new WaitUntilCommand(() -> barrels.isAtAngle),
        new ConditionalCommand(
            new InstantCommand(() -> WantedState = wantedState.IDLE),
            new InstantCommand(),
            () -> WantedState != wantedState.SHOOT_ALL),
        new ConditionalCommand(
            new InstantCommand(() -> isRunningCommand = false),
            new InstantCommand(),
            () -> WantedState != wantedState.SHOOT_ALL));
  }

  public void sendDriveInfoToRotation() {
    rotation.setRobotAngle(drive.getRotation());
    rotation.setRobotVelo(drive.getYawVelocity());
  }

  public void setWantedState(wantedState WantedState) {
    this.WantedState = WantedState;
  }
}
