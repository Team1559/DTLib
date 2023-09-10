package org.victorrobotics.frc.dtlib.logging;

import org.victorrobotics.frc.dtlib.DTSubsystem;

import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

public class DTLog {
  private static final SimpleDateFormat TIMESTAMP_FORMAT = new SimpleDateFormat(
      "MM-dd-yyyy_HH-mm-ss");
  private static final String           LOG_DIRECTORY    = "";

  private final Map<DTSubsystem, DTSubsystemLog> subsystemLogs;
  private final DataOutputStream                 dataStream;

  public DTLog() {
    subsystemLogs = new HashMap<>();
    try {
      dataStream = new DataOutputStream(new BufferedOutputStream(
          new FileOutputStream(new File(LOG_DIRECTORY, generateFilename()))));
    } catch (FileNotFoundException e) {
      throw new IllegalStateException("Failed to initialize log file");
    }
  }

  public void addSubsystem(DTSubsystem subsystem) {
    subsystemLogs.computeIfAbsent(subsystem, this::newSubsystemLog);
  }

  public void initialize() {
    writeHeader();
    for (Map.Entry<DTSubsystem, DTSubsystemLog> entry : subsystemLogs.entrySet()) {
      entry.getValue()
           .writeFormat(dataStream);
    }
  }

  public void log() {

  }

  private void writeHeader() {

  }

  private String generateFilename() {
    return TIMESTAMP_FORMAT.format(new Date()) + ".dtlog";
  }

  private DTSubsystemLog newSubsystemLog(DTSubsystem subsystem) {
    return new DTSubsystemLog(this, subsystem);
  }
}
