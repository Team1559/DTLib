package org.victorrobotics.dtlib.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AbsoluteEncoder {
  Object getEncoderImpl();

  Rotation2d getPosition();

  Rotation2d getAbsolutePosition();

  Rotation2d getVelocity();

  String getFirmwareVersion();

  AbsoluteEncoderFaults getFaults();

  boolean isInverted();

  void setRange(boolean signed);

  void setInverted(boolean invert);

  void setPosition(Rotation2d position);

  void setZeroPosition(Rotation2d position);

  void zeroPosition();
}
