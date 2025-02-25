package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;


public interface GyroIO {

  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    @Override
    public void toLog(LogTable table) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'toLog'");
    }
    @Override
    public void fromLog(LogTable table) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void setYaw(double degrees) {}
}
