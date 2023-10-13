package org.victorrobotics.dtlib.hardware;

/**
 * An interface representing a fault field for a {@link AbsoluteEncoder}.
 */
public interface AbsoluteEncoderFaults {
  /**
   * @return true if the device is suffering from a low-voltage type fault.
   */
  boolean lowVoltage();

  /**
   * @return true if the device is suffering from a hardware failure.
   */
  boolean hardwareFailure();

  /**
   * @return true if the device has declared any faults.
   */
  boolean hasAnyFault();

  /**
   * @return true if the device is reporting a fault that doesn't fall into a
   *         category specified here.
   */
  boolean other();
}
