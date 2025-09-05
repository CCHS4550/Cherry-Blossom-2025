package frc.robot.ControlSchemes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.wantedState;

/** how our controller or button board interacts with the superstructure */
public class MechanismScheme {

  /** creates the control scheme */
  public static void Configure(Superstructure superstructure, CommandXboxController controller) {
    configureButtons(superstructure, controller);
  }

  /** sets button bindings */
  public static void configureButtons(CommandXboxController controller) {

    // go up on DPAD UP
    controller
        .povUp()
        .whileTrue(
            new RunCommand(
                () -> elevation.setVoltage(1));
    controller
        .povUp()
        .whileFalse(new RunCommand ()-> elevation.setVoltage(0));

    // go down on DPAD DOWN
    controller
        .povDown()
        .whileTrue(new RunCommand (()-> elevation.setVoltage(-1));
    controller
        .povDown()
        .whileFalse(new RunCommand (()-> elevation.setVoltage(0)));

    // go right on DPAD right
    controller
        .povRight()
        .whileTrue (new RunCommand (()-> rotation.setVoltage (1));
    controller
        .povRight()
        .whileFalse(new RunCommand (()-> rotation.setVoltage (0)));

    // go left on DPAD left
    controller
        .povLeft()
        .whileTrue(new RunCommand (()-> rotation.setVoltage(-1)));
    controller
        .povLeft()
        .whileFalse(new RunCommand (()-> rotation.setVoltage(0)));

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
