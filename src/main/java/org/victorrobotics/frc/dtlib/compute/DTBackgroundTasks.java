package org.victorrobotics.frc.dtlib.compute;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.hal.NotifierJNI;

public class DTBackgroundTasks {
    private static final BlockingQueue<Runnable> TASK_QUEUE = new LinkedBlockingQueue<>();
    private static final ExecutorService         EXECUTOR   = new ThreadPoolExecutor(1, 1, 0, TimeUnit.MILLISECONDS,
            TASK_QUEUE);

    private static final Object MONITOR         = new Object();
    private static final int    NOTIFIER_HANDLE = NotifierJNI.initializeNotifier();

    private static volatile boolean shouldPause;
    private static volatile boolean isPaused;

    private DTBackgroundTasks() {}

    public static <T> DTBackgroundFuture<T> submit(DTBackgroundTask<T> task) {
        return new DTBackgroundFuture<>(task, EXECUTOR.submit(task));
    }

    public static void run(long timeoutMicros) {
        executeTasks(timeoutMicros);
        NotifierJNI.updateNotifierAlarm(NOTIFIER_HANDLE, timeoutMicros);
        NotifierJNI.waitForNotifierAlarm(NOTIFIER_HANDLE);
    }

    private static void executeTasks(long timeoutMicros) {
        if (TASK_QUEUE.isEmpty()) {
            return;
        }

        NotifierJNI.updateNotifierAlarm(NOTIFIER_HANDLE, timeoutMicros - 1000);
        shouldPause = false;
        MONITOR.notifyAll();
        NotifierJNI.waitForNotifierAlarm(NOTIFIER_HANDLE);
        shouldPause = true;
        while (!isPaused) {
            // wait
        }
    }

    static void pausePoint() {
        while (shouldPause) {
            isPaused = true;
            try {
                MONITOR.wait();
            } catch (InterruptedException e) {
                // ignore?
            }
        }
        isPaused = false;
    }
}
