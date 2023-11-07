package org.victorrobotics.dtlib.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An interface denoting hardware that supplies an absolutely-referenced
 * rotational position.
 */
public interface AbsoluteEncoder {
  /**
   * @return the underlying vendor implementation of this hardware, used to send
   *           and receive data.
   */
  Object getEncoderImpl();

  /**
   * @return the current position, relative to the configured offset.
   * @see #setPosition(Rotation2d)
   * @see #setZeroPosition(Rotation2d)
   */
  Rotation2d getPosition();

  /**
   * @return the current position, absolutely referenced (no offset)
   * @see #getPosition()
   */
  Rotation2d getAbsolutePosition();

  /**
   * @return the current rotational velocity (per second)
   */
  Rotation2d getVelocity();

  /**
   * @return a human-readable string representing the hardware firmware version
   */
  String getFirmwareVersion();

  /**
   * @return the faults currently reported by the hardware
   */
  AbsoluteEncoderFaults getFaults();

  /**
   * @return true if the hardware is reporting angles that are
   *           clockwise-positive
   * @see #setInverted(boolean)
   */
  boolean isInverted();

  /**
   * Determines whether absolute angles will be within the range [0º, 360º) or
   * [-180º, 180º)
   *
   * @param signed whether to use a signed output range
   */
  void setRange(boolean signed);

  /**
   * Sets the rotational direction that will increase the reported angle.
   *
   * @param invert true if angles shoulld be clockwise-positive
   * @see #isInverted()
   */
  void setInverted(boolean invert);

  /**
   * Sets the current hardware position to denote the provided virtual rotation,
   * applying an offset. This does not apply to absolute measurements.
   *
   * @param position the new position to set
   */
  void setPosition(Rotation2d position);

  /**
   * Sets the given position to an angle of zero. This has no impact on absolute
   * position.
   *
   * @param position the position which should be referenced as zero
   */
  void setZeroPosition(Rotation2d position);

  /**
   * Sets the current position to an angle of zero. This has no impact on
   * absolute position.
   *
   * @see #setPosition(Rotation2d)
   * @see #setZeroPosition(Rotation2d)
   */
  void zeroPosition();
}
