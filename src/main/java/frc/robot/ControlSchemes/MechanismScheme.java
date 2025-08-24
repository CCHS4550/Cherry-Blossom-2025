package frc.robot.ControlSchemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.wantedState;


/**
 * how our controller or button board interacts with the superstructure
 */
public class MechanismScheme {

  /**
   * creates the control scheme
   */
  public static void Configure(Superstructure superstructure, CommandXboxController controller) {
    configureButtons(superstructure, controller);
  }

  /** sets button bindings */
  public static void configureButtons(
      Superstructure superstructure, CommandXboxController controller) {

    // go up on DPAD UP
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setWantedState(wantedState.ELEVATION_OPENLOOP_UP)));
    controller
        .povUp()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    // go down on DPAD DOWN
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setWantedState(wantedState.ELEVATION_OPENLOOP_DOWN)));
    controller
        .povDown()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    // go right on DPAD right
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setWantedState(wantedState.ROTATION_OPENLOOP_CLOCKWISE)));
    controller
        .povRight()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    // go left on DPAD left
    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedState(wantedState.ROTATION_OPENLOOP_COUNTERCLOCKWISE)));
    controller
        .povLeft()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    // shoot one on right trigger
    controller
        .rightTrigger()
        .onTrue(new InstantCommand(() -> superstructure.setWantedState(wantedState.SHOOT_ONE)));

    // shoot all on both press
    controller
        .rightTrigger()
        .and(controller.leftTrigger())
        .onTrue(new InstantCommand(() -> superstructure.setWantedState(wantedState.SHOOT_ALL)));
  }
}
