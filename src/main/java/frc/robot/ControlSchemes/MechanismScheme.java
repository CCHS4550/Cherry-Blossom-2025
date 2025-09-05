package frc.robot.ControlSchemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.wantedState;
import org.littletonrobotics.junction.Logger;


/** how our controller or button board interacts with the superstructure */
public class MechanismScheme {

  /** creates the control scheme */
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
                () -> {
                    Logger.recordOutput("Elevation Open Loop Up Command", true);
                    superstructure.setWantedStatCommande(wantedState.ELEVATION_OPENLOOP_UP));
                });
    controller
        .povUp()
        .onFalse(new InstantCommand(() ->{
            Logger.recordOutput("Elevation Open Loop Up Command", false);
            superstructure.setWantedStateCommand(wantedState.IDLE));
        });

    // go down on DPAD DOWN
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                Logger.recordOutput("Elevation Open Loop Down Command", true);
                superstructure.setWantedStateCommand(wantedState.ELEVATION_OPENLOOP_DOWN));
                });
    controller
        .povDown()
        .onFalse(new InstantCommand(() -> {
            Logger.recordOutput("Elevation Open Loop Down Command", false);
            superstructure.setWantedStateCommand(wantedState.IDLE));
        });

    // go right on DPAD right
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                    Logger.recordOutput("Rotation Open Loop Clockwise Command", true);
                    superstructure.setWantedStateCommand(wantedState.ROTATION_OPENLOOP_CLOCKWISE));
                });
    controller
        .povRight()
        .onFalse(new InstantCommand(() -> {
            Logger.recordOutput("Rotation Open Loop Clockwise Command", false);
            superstructure.setWantedStateCommand(wantedState.IDLE));
        });

    // go left on DPAD left
    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () ->{
                    Logger.recordOutput("Elevation Open Loop Counterclockwise Command", true);
                    superstructure.setWantedStateCommand(wantedState.ROTATION_OPENLOOP_COUNTERCLOCKWISE));
                });
    controller
        .povLeft()
        .onFalse(new InstantCommand(() -> {
            Logger.recordOutput("Elevation Open Loop Counterclockwise Command", false);
            superstructure.setWantedStateCommand(wantedState.IDLE));
        });

    // shoot one on right trigger
    controller
        .rightTrigger()
        .onTrue(new InstantCommand(() -> {
            Logger.recordOutput("Shoot One Command", true);
            superstructure.setWantedStateCommand(wantedState.SHOOT_ONE));
        });

    // shoot all on both press
    controller
        .rightTrigger()
        .and(controller.leftTrigger())
        .onTrue(new InstantCommand(() -> {
            Logger.recordOutput("Shoot All Command", true);
            superstructure.setWantedStateCommand(wantedState.SHOOT_ALL));
        });

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                    Logger.recordOutput("Rotate 60 Degrees Command", true);
                    superstructure.setWantedStateCommand(wantedState.ROTATE_60_DEGREES_BOT_ORIENTED));
                });
  }
}
