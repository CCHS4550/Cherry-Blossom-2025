package frc.robot.Subsystems.Turret.Pneumatics;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

/**
 * creates a pneumatics system that handles bot pressure, as well as solenoids, while staying with
 * in safe limits
 *
 * <p>but also I don't know how pnuematics work
 */
public class Pneumatics extends SubsystemBase {

  private final PneumaticsIO
      io; // the interface used by pnuematics, will be defined as PneumaticsIOHardware if real

  private final PneumaticsIOInputsAutoLogged inputs =
      new PneumaticsIOInputsAutoLogged(); // logged inputs of the pneumatics system

  private double compressorPercent; // the percent that the compressor uses
  private double desiredPSI = 100.0; // the PSI we want to be at

  // potential bad practice
  public boolean isRunningCommand =
      false; // exists in order to prevent the periodic state machine from calling the same command
  // multiple times

  // state we want the pneumatics to be in
  public enum wantedPneumaticsState {
    TESTING,
    FILLING_AIR_TANK,
    SHOOT,
    IDLE
  }

  // state the pneumatics is in
  private enum SystemState {
    TESTING,
    FILLING_AIR_TANK,
    SHOOT,
    IDLE
  }

  // exists to prevent too many states/ overcrowd apply states
  // what testing task to perform
  private enum TestingTask {
    PRESSURE_SOLENOID_OFF,
    PRESSURE_SOLENOID_ON,
    SHOOTING_TRIGGER,
    NONE
  }

  // initialize our states
  wantedPneumaticsState WantedPneumaticsState = wantedPneumaticsState.IDLE;
  SystemState systemState = SystemState.IDLE;
  TestingTask testingTask = TestingTask.NONE;

  /**
   * constructor for the pneumatics
   *
   * @param io instance of PnuematicsIO or classes implementing PneumaticsIO
   */
  public Pneumatics(PneumaticsIO io) {
    this.io = io;
  }

  /**
   * will be called periodically
   *
   * <p>updating inputs and logging is NOT thread safe, so we use synchronized
   */
  @Override
  public void periodic() {
    // thread safe
    synchronized (inputs) {
      // update autologged inputs
      io.updateInputs(inputs);

      // stop the subsystem if disabled
      if (DriverStation.isDisabled()) {
        io.disablePressureSeal();
        io.setCompressor(0.0);
        systemState = SystemState.IDLE;
      }
      // set system state to match wanted state and deal with any changes that need to be made in
      // between states
      // should only be used when setting the shoot command sequence, and automaticall resets when
      // ever that is called
      systemState = handleStateTransitions();

      // turn the states into desired output
      applyStates();
    }
  }

  /**
   * sets the system state to be the same as the wanted state, but can be set to perform more
   * complex judgements on what state to goto if so desired
   *
   * @return the systemstate that our systemState variable will be set to
   */
  private SystemState handleStateTransitions() {

    return switch (WantedPneumaticsState) {
      case TESTING -> SystemState.TESTING;
      case FILLING_AIR_TANK -> {
        // check if at the desired PSI and less than the max PSI
        if (inputs.pressurePSI <= desiredPSI
            && inputs.pressurePSI < Constants.PneumaticConstants.maxPSI) {
          yield SystemState.FILLING_AIR_TANK; // keep filling if room to go
        } else {
          yield SystemState.IDLE; // stop if there or at max
        }
      }
      case SHOOT -> SystemState.SHOOT;
      case IDLE -> SystemState.IDLE;
      default -> SystemState.IDLE;
    };
  }

  // perform a desired outcome depending on our state
  private void applyStates() {
    switch (systemState) {
      case TESTING:
        doTestingTask().schedule();
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
          runShootingSequence().schedule(); // only shoot sequence when the previous one is done
        }
        break;
      case IDLE:
        break;
    }
  }

  /**
   * exists to not create too many state/over crowd apply states
   *
   * @return a command with the testing task to perform
   */
  private Command doTestingTask() {
    return switch (testingTask) {
      case PRESSURE_SOLENOID_ON -> new InstantCommand(() -> io.enablePressureSeal());
      case PRESSURE_SOLENOID_OFF -> new InstantCommand(() -> io.disablePressureSeal());
      case SHOOTING_TRIGGER -> new StartEndCommand(
              () -> io.setShootingSeal(true), () -> io.setShootingSeal(false))
          .withTimeout(0.2);
      case NONE -> null;
      default -> null;
    };
  }

  /**
   * the steps necesary to create a seal, shoot, and disable the seal to allow indexing
   *
   * <p>also automatically resets the back to idle when false so it can run again also automatically
   * resets the isRunningCommand variable so that shouldn't be manipulated elsewhat
   *
   * @return the squencial command group to follow run
   */
  private SequentialCommandGroup runShootingSequence() {
    isRunningCommand = true; // we are running a command
    return new SequentialCommandGroup(
        new InstantCommand(() -> io.enablePressureSeal()), // get the seal
        new WaitCommand(0.3), // wait to let it happen
        new StartEndCommand(() -> io.setShootingSeal(true), () -> io.setShootingSeal(false))
            .withTimeout(0.2), // shoot
        new WaitCommand(0.01),
        new InstantCommand(
            () -> io.disablePressureSeal()), // disable the pressure seal to allow indexing
        new WaitCommand(0.01),
        new InstantCommand(() -> isRunningCommand = false), // indicate the command is done
        new InstantCommand(
            () -> setWantedState(wantedPneumaticsState.IDLE))); // set our wanted state to idle
  }

  /**
   * sets the pneumatic's wanted state should be the primary way of manipulating the pneumatics
   * outside of the class
   *
   * @param wantedState the desired state
   */
  public void setWantedState(wantedPneumaticsState wantedPneumaticsState) {
    this.WantedPneumaticsState = wantedPneumaticsState;
  }

  /**
   * set the desired pressure of the air tank
   *
   * @param PSI the desired pressure in PSI
   */
  public void setDesiredPressure(double PSI) {
    desiredPSI = PSI;
  }
}
