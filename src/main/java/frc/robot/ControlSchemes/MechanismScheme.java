package frc.robot.ControlSchemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.wantedState;

public class MechanismScheme {

  public static void Configure(Superstructure superstructure, CommandXboxController controller) {
    configureButtons(superstructure, controller);
  }

  public static void configureButtons(
      Superstructure superstructure, CommandXboxController controller) {
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setWantedState(wantedState.ELEVATION_OPENLOOP_UP)));
    controller
        .povUp()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setWantedState(wantedState.ELEVATION_OPENLOOP_DOWN)));
    controller
        .povDown()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setWantedState(wantedState.ROTATION_OPENLOOP_CLOCKWISE)));
    controller
        .povRight()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedState(wantedState.ROTATION_OPENLOOP_COUNTERCLOCKWISE)));
    controller
        .povLeft()
        .onFalse(new InstantCommand(() -> superstructure.setWantedState(wantedState.IDLE)));

    controller
        .rightTrigger()
        .onTrue(new InstantCommand(() -> superstructure.setWantedState(wantedState.SHOOT_ONE)));

    controller
        .rightTrigger()
        .and(controller.leftTrigger())
        .onTrue(new InstantCommand(() -> superstructure.setWantedState(wantedState.SHOOT_ALL)));
  }
}
