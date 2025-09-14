package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.wantedState;
import frc.robot.Subsystems.Turret.Barrels.Barrel.wantedBarrelState;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics.wantedPneumaticsState;
// **this is all fucking horrible practice im just lazy right now ok its 12:34 am and I just want to
// sleep */

public class MechanismCommands {
  private MechanismCommands() {}

  public static Command shootingSequence(Pneumatics pnuematics) {
    Command shoot =
        new SequentialCommandGroup(
            new InstantCommand(() -> pnuematics.setWantedState(wantedPneumaticsState.SHOOT)),
            new InstantCommand(() -> pnuematics.isRunningCommand = true),
            new InstantCommand(() -> pnuematics.io.enablePressureSeal()), // get the seal
            new WaitCommand(0.3), // wait to let it happen
            new StartEndCommand(
                    () -> pnuematics.io.setShootingSeal(true),
                    () -> pnuematics.io.setShootingSeal(false))
                .withTimeout(0.2), // shoot
            new WaitCommand(0.01),
            new InstantCommand(
                () ->
                    pnuematics.io
                        .disablePressureSeal()), // disable the pressure seal to allow indexing
            new WaitCommand(0.01),
            new InstantCommand(
                () -> pnuematics.isRunningCommand = false), // indicate the command is done
            new InstantCommand(() -> pnuematics.setWantedState(wantedPneumaticsState.IDLE)));
    shoot.addRequirements(pnuematics);
    return shoot;
  }

  public static Command shootThenIndex(Superstructure superstructure) {
    Command shootThenIndex =
        new SequentialCommandGroup(
            new ConditionalCommand(
                new InstantCommand(() -> superstructure.WantedState = wantedState.SHOOT_ONE),
                new InstantCommand(),
                () -> superstructure.WantedState != wantedState.SHOOT_ALL),
            new InstantCommand(() -> superstructure.isRunningCommand = true),
            new InstantCommand(
                () ->
                    superstructure.pneumatics.setWantedState(
                        wantedPneumaticsState.SHOOT)), // shoot using the pneumatics state
            shootingSequence(superstructure.pneumatics),
            new WaitUntilCommand(
                () ->
                    !superstructure
                        .pneumatics
                        .isRunningCommand), // wait until that sequence has finishied
            new InstantCommand(
                () ->
                    superstructure.barrels.setWantedState(
                        wantedBarrelState.INDEX)), // index the barrel
            new WaitUntilCommand(
                () -> superstructure.barrels.isAtAngle), // wait until we are at angle, then move on

            // reset state if not in the SHOOT_ALL state, otherwise, return a null command
            new ConditionalCommand(
                new InstantCommand(() -> superstructure.WantedState = wantedState.IDLE),
                new InstantCommand(),
                () -> superstructure.WantedState != wantedState.SHOOT_ALL),

            // reset boolean if not in the SHOOT_ALL state, otherwise return a null command
            new ConditionalCommand(
                new InstantCommand(() -> superstructure.isRunningCommand = false),
                new InstantCommand(),
                () -> superstructure.WantedState != wantedState.SHOOT_ALL));
    shootThenIndex.addRequirements(superstructure);
    return shootThenIndex;
  }

  public static Command shootSix(Superstructure superstructure) {
    Command shootAll =
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    superstructure.WantedState =
                        wantedState.SHOOT_ALL), // got back to idle so his state can be called again
            new InstantCommand(() -> superstructure.isRunningCommand = true),
            shootThenIndex(superstructure),
            shootThenIndex(superstructure),
            shootThenIndex(superstructure),
            shootThenIndex(superstructure),
            shootThenIndex(superstructure),
            shootThenIndex(superstructure),
            new InstantCommand(
                () ->
                    superstructure.WantedState =
                        wantedState.IDLE), // got back to idle so this state can be called again
            new InstantCommand(() -> superstructure.isRunningCommand = false)); // reset the boolean
    shootAll.addRequirements(superstructure);
    return shootAll;
  }
}
