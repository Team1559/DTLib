package org.victorrobotics.frc.dtlib.compute;

import java.util.concurrent.Callable;

public interface DTBackgroundTask<T> extends Callable<T> {
    @Override
    T call();

    default DTBackgroundFuture<T> submit() {
        return DTBackgroundTasks.submit(this);
    }

    static void pausePoint() {
        DTBackgroundTasks.pausePoint();
    }
}
