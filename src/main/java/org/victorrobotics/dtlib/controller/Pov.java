package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.CommandScheduler;

import java.util.function.IntSupplier;

public class Pov implements IntSupplier {
  private static final double SQRT_2_DIV_2 = Math.sqrt(2) * 0.5;

  private final IntSupplier supplier;

  private int value;

  public Pov(IntSupplier supplier) {
    this.supplier = supplier;
    CommandScheduler.bindCallback(this::refresh);
    refresh();
  }

  private void refresh() {
    value = supplier.getAsInt();
  }

  @Override
  public int getAsInt() {
    return value;
  }

  public Trigger center() {
    return new Trigger(() -> value == -1);
  }

  public Trigger up() {
    return new Trigger(() -> value == 0);
  }

  public Trigger upRight() {
    return new Trigger(() -> value == 45);
  }

  public Trigger right() {
    return new Trigger(() -> value == 90);
  }

  public Trigger downRight() {
    return new Trigger(() -> value == 135);
  }

  public Trigger down() {
    return new Trigger(() -> value == 180);
  }

  public Trigger downLeft() {
    return new Trigger(() -> value == 225);
  }

  public Trigger left() {
    return new Trigger(() -> value == 270);
  }

  public Trigger upLeft() {
    return new Trigger(() -> value == 315);
  }

  public Axis xAxis() {
    return new Axis(() -> switch (supplier.getAsInt()) {
      case 90 -> 1D;
      case 45, 135 -> SQRT_2_DIV_2;
      case 225, 315 -> -SQRT_2_DIV_2;
      case 270 -> -1D;
      default -> 0D;
    });
  }

  public Axis yAxis() {
    return new Axis(() -> switch (supplier.getAsInt()) {
      case 0 -> 1D;
      case 315, 45 -> SQRT_2_DIV_2;
      case 135, 225 -> -SQRT_2_DIV_2;
      case 180 -> -1D;
      default -> 0D;
    });
  }
}
