package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ControlSchemes.DriveScheme;
import frc.robot.ControlSchemes.MechanismScheme;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.Gyro.GyroIO;
import frc.robot.Subsystems.Drive.Gyro.GyroPigeon;
import frc.robot.Subsystems.Drive.Module.ModuleIO;
import frc.robot.Subsystems.Drive.Module.ModuleIOSpark;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Turret.Barrels.Barrel;
import frc.robot.Subsystems.Turret.Barrels.BarrelIO;
import frc.robot.Subsystems.Turret.Barrels.BarrelIOSpark;
import frc.robot.Subsystems.Turret.Elevation.Elevation;
import frc.robot.Subsystems.Turret.Elevation.ElevationIO;
import frc.robot.Subsystems.Turret.Elevation.ElevationIOSpark;
import frc.robot.Subsystems.Turret.Pneumatics.Pneumatics;
import frc.robot.Subsystems.Turret.Pneumatics.PneumaticsIO;
import frc.robot.Subsystems.Turret.Pneumatics.PneumaticsIOHardware;
import frc.robot.Subsystems.Turret.Rotation.Rotation;
import frc.robot.Subsystems.Turret.Rotation.RotationIO;
import frc.robot.Subsystems.Turret.Rotation.RotationIOSpark;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOPhotonvision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // subclasses of the robot
  private final Drive drive;
  private final Vision vision;
  private final Barrel barrels;
  private final Elevation elevation;
  private final Pneumatics pneumatics;
  private final Rotation rotation;

  // superstructure
  private final Superstructure superstructure;

  // controller used
  CommandXboxController primaryController = new CommandXboxController(0);

  // autochooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroPigeon(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        barrels = new Barrel(new BarrelIOSpark());
        elevation = new Elevation(new ElevationIOSpark());
        pneumatics = new Pneumatics(new PneumaticsIOHardware());
        rotation = new Rotation(new RotationIOSpark());

        vision =
            new Vision(
                drive,
                new VisionIOPhotonvision("Camera 1", cameraOneToRobot),
                new VisionIOPhotonvision("Camera 2", cameraTwoToRobot));

        // create the super structure
        superstructure = new Superstructure(pneumatics, barrels, elevation, rotation, drive);

        // configure control schemes
        DriveScheme.configure(drive, primaryController);
        MechanismScheme.Configure(superstructure, primaryController);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        barrels = new Barrel(new BarrelIO() {});
        elevation = new Elevation(new ElevationIO() {});
        pneumatics = new Pneumatics(new PneumaticsIO() {});
        rotation = new Rotation(new RotationIO() {});

        vision = new Vision(drive, new VisionIO() {});

        superstructure = new Superstructure(pneumatics, barrels, elevation, rotation, drive);

        DriveScheme.configure(drive, primaryController);
        MechanismScheme.Configure(superstructure, primaryController);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        barrels = new Barrel(new BarrelIO() {});
        elevation = new Elevation(new ElevationIO() {});
        pneumatics = new Pneumatics(new PneumaticsIO() {});
        rotation = new Rotation(new RotationIO() {});

        vision = new Vision(drive, new VisionIO() {});

        superstructure = new Superstructure(pneumatics, barrels, elevation, rotation, drive);
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
