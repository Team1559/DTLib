package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.DTCommandScheduler;

import java.util.function.IntSupplier;

public class DTPov implements IntSupplier {
  private final IntSupplier supplier;

  private int value;

  public DTPov(IntSupplier supplier) {
    this.supplier = supplier;
    DTCommandScheduler.bindInputCallback(this::refresh);
  }

  private void refresh() {
    value = supplier.getAsInt();
  }

  @Override
  public int getAsInt() {
    return value;
  }

  private DTTrigger makeTrigger(int dir) {
    return new DTTrigger(() -> value == dir);
  }

  public DTTrigger center() {
    return makeTrigger(-1);
  }

  public DTTrigger up() {
    return makeTrigger(0);
  }

  public DTTrigger upRight() {
    return makeTrigger(45);
  }

  public DTTrigger right() {
    return makeTrigger(90);
  }

  public DTTrigger downRight() {
    return makeTrigger(135);
  }

  public DTTrigger down() {
    return makeTrigger(180);
  }

  public DTTrigger downLeft() {
    return makeTrigger(225);
  }

  public DTTrigger left() {
    return makeTrigger(270);
  }

  public DTTrigger upLeft() {
    return makeTrigger(315);
  }

  public DTAxis xAxis() {
    return new DTAxis(() -> {
      switch (supplier.getAsInt()) {
        case 45:
        case 90:
        case 135:
          return 1D;
        case 225:
        case 270:
        case 315:
          return -1D;
        default:
          return 0D;
      }
    });
  }

  public DTAxis yAxis() {
    return new DTAxis(() -> {
      switch (supplier.getAsInt()) {
        case 315:
        case 0:
        case 45:
          return 1D;
        case 135:
        case 180:
        case 225:
          return -1D;
        default:
          return 0D;
      }
    });
  }
}
