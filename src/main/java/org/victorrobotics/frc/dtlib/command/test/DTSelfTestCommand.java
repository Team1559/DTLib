package org.victorrobotics.frc.dtlib.command.test;

import org.victorrobotics.frc.dtlib.DTSubsystem;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;
import org.victorrobotics.frc.dtlib.command.util.DTPrintCommand;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class DTSelfTestCommand extends DTCommandBase implements DTTestCommand {
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
    protected void start() {
        CommandScheduler.getInstance()
                        .schedule(new DTPrintCommand("Starting self test"));
        currentTestIndex = -1;
    }

    @Override
    protected void run() {
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
    protected void finish(boolean interrupted) {
        if (interrupted) {
            CommandScheduler.getInstance()
                            .schedule(new DTPrintCommand(String.format("Self test interrupted during test %d (%s)",
                                    currentTestIndex + 1, currentTest().subsystem.getName())));
        } else {
            CommandScheduler.getInstance()
                            .schedule(new DTPrintCommand("Self test finished"));
        }
        currentTestIndex = -1;
    }

    @Override
    protected boolean isComplete() {
        return currentTestIndex == tests.length;
    }

    public SelfTest[] getResults() {
        return tests;
    }

    public static class SelfTest {
        public final DTSubsystem   subsystem;
        public final DTTestCommand testCommand;
        public boolean             isFinished;
        public boolean             wasSuccessful;

        SelfTest(DTSubsystem subsystem) {
            this.subsystem = subsystem;
            testCommand = subsystem.getSelfTestCommand();
            wasSuccessful = false;
        }
    }
}
