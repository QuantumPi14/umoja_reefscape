package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ThirftyEncoder {
private AnalogInput encoder;
private Double offset = new Rotation2d();

    public ThirftyEncoder(int id) {
        encoder = new AnalogInput(id);
    }

    public void setOffset(Double offset) {
        this.offset = offset;
    }

    public double getRawAbsolutePosition() {
        return encoder.getAverageVoltage() / RobotController.getVoltage5V();
}

    public Rotation2d getAbsolutePosition() {
        double angle = Math.toRadians(360.0 * getRawAbsolutePosition() - offset.getDegrees());
        if (angle < 0) {
            angle = Math.PI * 2 + angle;
        }
        return Rotation2d.fromRadians(angle);
    }
}