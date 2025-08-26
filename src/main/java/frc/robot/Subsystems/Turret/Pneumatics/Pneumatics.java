package frc.robot.Subsystems.Turret.Pneumatics;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

  private final PneumaticsIO io;

  private final PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();

  private double compressorPercent;
  private double desiredPSI = 100.0;

  private boolean isRunningCommand = false;

  public enum wantedPneumaticsState {
    TESTING,
    FILLING_AIR_TANK,
    SHOOT,
    IDLE
  }

  private enum SystemState {
    TESTING,
    FILLING_AIR_TANK,
    SHOOT,
    IDLE
  }

  private enum TestingTask {
    PRESSURE_SOLENOID_OFF,
    PRESSURE_SOLENOID_ON,
    SHOOTING_TRIGGER,
    NONE
  }

  wantedPneumaticsState WantedPneumaticsState = wantedPneumaticsState.IDLE;
  SystemState systemState = SystemState.IDLE;
  TestingTask testingTask = TestingTask.NONE;

  public Pneumatics(PneumaticsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    synchronized (inputs) {
      io.updateInputs(inputs);

      if (DriverStation.isDisabled()) {
        io.disablePressureSeal();
        io.setCompressor(0.0);
        systemState = SystemState.IDLE;
      }
      systemState = handleStateTransitions();
      applyStates();
    }
  }

  private SystemState handleStateTransitions() {

    return switch (WantedPneumaticsState) {
      case TESTING -> SystemState.TESTING;
      case FILLING_AIR_TANK -> {
        if (inputs.pressurePSI <= desiredPSI
            && inputs.pressurePSI < Constants.PneumaticConstants.maxPSI) {
          yield SystemState.FILLING_AIR_TANK;
        } else {
          yield SystemState.IDLE;
        }
      }
      case SHOOT -> SystemState.SHOOT;
      case IDLE -> SystemState.IDLE;
      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case TESTING:
        doTestingTask();
        break;
      case FILLING_AIR_TANK:
        if (inputs.pressurePSI <= desiredPSI
            && inputs.pressurePSI
                < Constants.PneumaticConstants
                    .maxPSI) { // run the if statement again just for redundency
          io.setCompressor(compressorPercent);
        }
        break;
      case SHOOT:
        if (!isRunningCommand) {
          runShootingSequence();
        }
        break;
      case IDLE:
        break;
    }
  }

  private Command doTestingTask() {
    return switch (testingTask) {
      case PRESSURE_SOLENOID_OFF -> new InstantCommand(() -> io.enablePressureSeal());
      case PRESSURE_SOLENOID_ON -> new InstantCommand(() -> io.disablePressureSeal());
      case SHOOTING_TRIGGER -> new StartEndCommand(
              () -> io.setShootingSeal(true), () -> io.setShootingSeal(false))
          .withTimeout(0.2);
      case NONE -> null;
      default -> null;
    };
  }

  private SequentialCommandGroup runShootingSequence() {
    isRunningCommand = true;
    return new SequentialCommandGroup(
        new InstantCommand(() -> io.enablePressureSeal()),
        new WaitCommand(0.3),
        new StartEndCommand(() -> io.setShootingSeal(true), () -> io.setShootingSeal(false))
            .withTimeout(0.2),
        new WaitCommand(0.01),
        new InstantCommand(() -> io.disablePressureSeal()),
        new WaitCommand(0.01),
        new InstantCommand(() -> isRunningCommand = false),
        new InstantCommand(() -> setWantedState(wantedPneumaticsState.IDLE)));
  }

  public void setWantedState(wantedPneumaticsState wantedPneumaticsState) {
    this.WantedPneumaticsState = wantedPneumaticsState;
  }

  public void setDesiredPressure(double PSI) {
    desiredPSI = PSI;
  }
}
