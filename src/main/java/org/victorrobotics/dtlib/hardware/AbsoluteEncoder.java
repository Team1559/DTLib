package org.victorrobotics.dtlib.hardware;

import org.victorrobotics.dtlib.network.DTSendable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * An interface denoting hardware that supplies an absolutely-referenced
 * rotational position.
 */
public interface AbsoluteEncoder extends DTSendable {
  /**
   * @return the underlying vendor implementation of this hardware, used to send
   *         and receive data.
   */
  Object getEncoderImpl();

  /**
   * @return the current position, relative to the configured offset.
   *
   * @see #setPosition(Rotation2d)
   * @see #setZeroPosition(Rotation2d)
   */
  Rotation2d getPosition();

  /**
   * @return the current position, absolutely referenced (no offset)
   *
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
   *         clockwise-positive
   *
   * @see #setInverted(boolean)
   */
  boolean isInverted();

  /**
   * Determines whether absolute angles will be within the range [0º, 360º) or
   * [-180º, 180º)
   *
   * @param signed
   *        whether to use a signed output range
   */
  void setRange(boolean signed);

  /**
   * Sets the rotational direction that will increase the reported angle.
   *
   * @param invert
   *        true if angles shoulld be clockwise-positive
   *
   * @see #isInverted()
   */
  void setInverted(boolean invert);

  /**
   * Sets the current hardware position to denote the provided virtual rotation,
   * applying an offset. This does not apply to absolute measurements.
   *
   * @param position
   *        the new position to set
   */
  void setPosition(Rotation2d position);

  /**
   * Sets the given position to an angle of zero. This has no impact on absolute
   * position.
   *
   * @param position
   *        the position which should be referenced as zero
   */
  void setZeroPosition(Rotation2d position);

  /**
   * Sets the current position to an angle of zero. This has no impact on absolute position.
   *
   * @see #setPosition(Rotation2d)
   * @see #setZeroPosition(Rotation2d)
   */
  void zeroPosition();

  @Override
  void close();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position",
                              limitRate(() -> getPosition().getDegrees(), UPDATE_RATE_FAST_HZ),
                              d -> setPosition(Rotation2d.fromDegrees(d)));
    builder.addDoubleProperty("Absolute", limitRate(() -> getAbsolutePosition().getDegrees(),
                                                    UPDATE_RATE_FAST_HZ),
                              null);

    builder.addBooleanProperty("Inverted", limitRate(this::isInverted, UPDATE_RATE_SLOW_HZ),
                               this::setInverted);
    builder.addStringProperty("Firmware", limitRate(this::getFirmwareVersion, UPDATE_RATE_FAST_HZ),
                              null);
  }
}
