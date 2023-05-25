package org.victorrobotics.frc.dtlib.compute;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

public class DTBackgroundFuture<T> {
    private final DTBackgroundTask<T> task;
    private final Future<T>           future;

    public DTBackgroundFuture(DTBackgroundTask<T> task, Future<T> future) {
        this.task = task;
        this.future = future;
    }

    public DTBackgroundTask<T> getTask() {
        return task;
    }

    public T getResult() {
        return getResultElse(null);
    }

    public T getResultElse(T defaultValue) {
        if (future.isDone() && !future.isCancelled()) {
            try {
                return future.get();
            } catch (InterruptedException | CancellationException | ExecutionException e) {
                // return defaultValue
            }
        }
        return defaultValue;
    }

    public boolean isDone() {
        return future.isDone();
    }
}
