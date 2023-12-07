package org.victorrobotics.dtlib.subsystem.swerve;

import static java.util.Objects.requireNonNull;

import org.victorrobotics.dtlib.hardware.AbsoluteEncoder;
import org.victorrobotics.dtlib.hardware.Motor;
import org.victorrobotics.dtlib.math.geometry.Vector2D;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A Swerve Drive Specialties MK4 or MK4i module.
 */
public class SdsMk4Module implements SwerveModule {
  public enum GearRatio {
    L1(57D / 7D), // 8.14285714
    L2(425D / 63D), // 6.74603175
    L3(300D / 49D), // 6.12244898
    L4(36D / 7D); // 5.14285714

    public final double value;

    GearRatio(double value) {
      this.value = value;
    }
  }

  private static final double STEER_GEAR_RATIO    = 64D / 5D;
  private static final double WHEEL_CIRCUMFERENCE = 4 * 0.0254 * Math.PI;

  private final Motor           driveMotor;
  private final Motor           steerMotor;
  private final AbsoluteEncoder steerEncoder;

  private final Vector2D  location;
  private final GearRatio driveGearRatio;

  public SdsMk4Module(Motor driveMotor, Motor steerMotor, AbsoluteEncoder encoder,
                      GearRatio driveGearRatio, Vector2D location) {
    this.driveMotor = requireNonNull(driveMotor);
    this.steerMotor = requireNonNull(steerMotor);
    this.steerEncoder = requireNonNull(encoder);

    this.location = location;
    this.driveGearRatio = driveGearRatio;

    steerMotor.setPosition(encoder.getPosition()
                                  .getRotations());
  }

  @Override
  public void setSpeed(double metersPerSecond) {
    double rpm = metersPerSecond * driveGearRatio.value / WHEEL_CIRCUMFERENCE * 60;
    System.out.println(rpm);
    driveMotor.setVelocity(metersPerSecond * driveGearRatio.value / WHEEL_CIRCUMFERENCE * 60);
  }

  @Override
  public double getSpeed() {
    return driveMotor.getVelocityRPM() * WHEEL_CIRCUMFERENCE / driveGearRatio.value / 60;
  }

  @Override
  public void setAngle(Rotation2d angle) {
    steerMotor.setPosition(angle.getRotations() * STEER_GEAR_RATIO);
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(steerMotor.getEncoderPosition() / STEER_GEAR_RATIO);
  }

  @Override
  public double getDistance() {
    return driveMotor.getEncoderPosition() * WHEEL_CIRCUMFERENCE / driveGearRatio.value / 60;
  }

  @Override
  public Vector2D getLocation() {
    return location;
  }

  @Override
  public double maxSpeed() {
    return driveMotor.getMaxVelocity() * WHEEL_CIRCUMFERENCE / driveGearRatio.value / 60;
  }
}
