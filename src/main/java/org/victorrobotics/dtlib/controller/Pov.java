package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.CommandScheduler;

import java.util.function.IntSupplier;

public class DTPov implements IntSupplier {
  private static final double SQRT_2_DIV_2 = Math.sqrt(2) * 0.5;

  private final IntSupplier supplier;

  private int value;

  public DTPov(IntSupplier supplier) {
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

  public DTTrigger center() {
    return new DTTrigger(() -> value == -1);
  }

  public DTTrigger up() {
    return new DTTrigger(() -> value == 0);
  }

  public DTTrigger upRight() {
    return new DTTrigger(() -> value == 45);
  }

  public DTTrigger right() {
    return new DTTrigger(() -> value == 90);
  }

  public DTTrigger downRight() {
    return new DTTrigger(() -> value == 135);
  }

  public DTTrigger down() {
    return new DTTrigger(() -> value == 180);
  }

  public DTTrigger downLeft() {
    return new DTTrigger(() -> value == 225);
  }

  public DTTrigger left() {
    return new DTTrigger(() -> value == 270);
  }

  public DTTrigger upLeft() {
    return new DTTrigger(() -> value == 315);
  }

  public DTAxis xAxis() {
    return new DTAxis(() -> switch (supplier.getAsInt()) {
      case 90 -> 1D;
      case 45, 135 -> SQRT_2_DIV_2;
      case 225, 315 -> -SQRT_2_DIV_2;
      case 270 -> -1D;
      default -> 0D;
    });
  }

  public DTAxis yAxis() {
    return new DTAxis(() -> switch (supplier.getAsInt()) {
      case 0 -> 1D;
      case 315, 45 -> SQRT_2_DIV_2;
      case 135, 225 -> -SQRT_2_DIV_2;
      case 180 -> -1D;
      default -> 0D;
    });
  }
}
