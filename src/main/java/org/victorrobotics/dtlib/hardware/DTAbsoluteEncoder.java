package org.victorrobotics.dtlib.hardware;

import org.victorrobotics.dtlib.network.DTSendable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTAbsoluteEncoder extends DTSendable {
  Object getEncoderImpl();

  Rotation2d getPosition();

  Rotation2d getAbsolutePosition();

  Rotation2d getVelocity();

  String getFirmwareVersion();

  DTAbsoluteEncoderFaults getFaults();

  boolean isInverted();

  void setRange(boolean signed);

  void setInverted(boolean invert);

  void setPosition(Rotation2d position);

  void setZeroPosition(Rotation2d position);

  void zeroPosition();

  @Override
  void close();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position",
        limitRate(() -> getPosition().getDegrees(), UPDATE_RATE_FAST_HZ),
        d -> setPosition(Rotation2d.fromDegrees(d)));
    builder.addDoubleProperty("Absolute",
        limitRate(() -> getAbsolutePosition().getDegrees(), UPDATE_RATE_FAST_HZ), null);

    builder.addBooleanProperty("Inverted", limitRate(this::isInverted, UPDATE_RATE_SLOW_HZ),
        this::setInverted);
    builder.addStringProperty("Firmware", limitRate(this::getFirmwareVersion, UPDATE_RATE_FAST_HZ),
        null);
  }
}
