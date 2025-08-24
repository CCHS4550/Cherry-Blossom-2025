package frc.robot.ControlSchemes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Drive.WantedState;
import java.util.function.DoubleSupplier;

/**
 * how a controller interacts with the drive train
 */
public class DriveScheme {
  // slow mode or fast mode
  private static DoubleSupplier driveSpeedModifier = () -> 0.4;

  public static void configure(Drive drive, CommandXboxController controller) {
    // default command will periodically run in drive train, in this case it periodically updates our joystick values
    drive.setDefaultCommand(
        new ParallelCommandGroup(
            Commands.run(
                () ->
                    drive.setXJoystickInput(
                        controller.getLeftX() * driveSpeedModifier.getAsDouble())),
            Commands.run(
                () ->
                    drive.setYJoystickInput(
                        controller.getLeftY() * driveSpeedModifier.getAsDouble())),
            Commands.run(
                () ->
                    drive.setOmegaJoystickInput(
                        controller.getRightX() * driveSpeedModifier.getAsDouble()))));

    // set button bindings
    configureButtons(controller, drive);
  }

  // sets button bindings
  private static void configureButtons(CommandXboxController controller, Drive drive) {
    
    // slow mode and fast mode
    controller.rightBumper().onTrue(Commands.runOnce(() -> setFastMode()));
    controller.rightBumper().onFalse(Commands.runOnce(() -> setSlowMode()));

    // drive to point while button is held
    controller.a().whileTrue(Commands.run(() -> drive.setWantedState(WantedState.DRIVE_TO_POINT)));
    controller.a().onFalse(Commands.run(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // path on the fly while button is held
    controller.b().whileTrue(Commands.run(() -> drive.setWantedState(WantedState.PATH_ON_THE_FLY)));
    controller.b().onFalse(Commands.run(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));

    // drive at angle while button is held
    controller
        .x()
        .whileTrue(Commands.run(() -> drive.setWantedState(WantedState.TELEOP_DRIVE_AT_ANGLE)));
    controller.x().onFalse(Commands.run(() -> drive.setWantedState(WantedState.TELEOP_DRIVE)));
  }

  // setters for fast and slow mode
  public static void setFastMode() {
    driveSpeedModifier = () -> 1.0;
  }

  public static void setSlowMode() {
    driveSpeedModifier = () -> 0.4;
  }
}
