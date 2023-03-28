package org.victorrobotics.frc.dtlib.command.test;

import org.victorrobotics.frc.dtlib.DTSubsystem;
import org.victorrobotics.frc.dtlib.command.util.DTPrintCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class DTSelfTestCommand extends CommandBase implements DTTestCommand {
    private final SelfTest[] tests;
    private int              currentTestIndex;

    public DTSelfTestCommand(DTSubsystem... subsystems) {
        tests = new SelfTest[subsystems.length];
        currentTestIndex = -1;
        for (int i = 0; i < subsystems.length; i++) {
            tests[i] = new SelfTest(subsystems[i]);
        }
    }

    private SelfTest currentTest() {
        return tests[currentTestIndex];
    }

    private void startNextTest() {
        CommandScheduler.getInstance()
                        .schedule(currentTest().testCommand);
    }

    private void recordTestResult() {
        currentTest().isFinished = currentTest().testCommand.isFinished();
        currentTest().wasSuccessful = currentTest().testCommand.wasSuccessful();
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance()
                        .schedule(new DTPrintCommand("Starting self test"));
        currentTestIndex = -1;
    }

    @Override
    public void execute() {
        if (isFinished()) {
            return;
        } else if (currentTestIndex == -1) {
            currentTestIndex = 0;
            startNextTest();
        }

        if (!currentTest().testCommand.isScheduled()) {
            recordTestResult();
            currentTestIndex++;
            if (currentTestIndex < tests.length) {
                startNextTest();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            CommandScheduler.getInstance()
                            .schedule(new DTPrintCommand(String.format(
                                    "Self test interrupted during test %d (%s)",
                                    currentTestIndex + 1, currentTest().subsystem.getName())));
        } else {
            CommandScheduler.getInstance()
                            .schedule(new DTPrintCommand("Self test finished"));
        }
        currentTestIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return currentTestIndex == tests.length;
    }

    public SelfTest[] getResults() {
        return tests;
    }

    private static class SelfTest {
        final DTSubsystem   subsystem;
        final DTTestCommand testCommand;
        boolean             isFinished;
        boolean             wasSuccessful;

        SelfTest(DTSubsystem subsystem) {
            this.subsystem = subsystem;
            testCommand = subsystem.getSelfTestCommand();
            wasSuccessful = false;
        }
    }
}
