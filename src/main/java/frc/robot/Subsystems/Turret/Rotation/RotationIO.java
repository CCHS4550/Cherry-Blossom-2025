package frc.robot.Subsystems.Turret.Rotation;

import org.littletonrobotics.junction.AutoLog;


public interface RotationIO {

    @AutoLog
    public static class RotationIOInputs{
        public boolean rotationConnected = false;
        public double rotationPositionRad = 0.0;
        public double rotationVelocityRadPerSec = 0.0;
        public double rotationAppliedVolts = 0.0;
        public double rotationCurrentAmps = 0.0;
    }

    public default void updateInputs(RotationIOInputs inputs) {}

    public default void setRotationOpenLoop(){}

    public default void setRotationPos(){}
}
