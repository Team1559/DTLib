package org.victorrobotics.frc.dtlib.network;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.SendableRegistry;

public class DTSendableChooser<T> implements NTSendable, AutoCloseable {
    /** The key for the default value. */
    private static final String DEFAULT  = "default";
    /** The key for the selected option. */
    private static final String SELECTED = "selected";
    /** The key for the active option. */
    private static final String ACTIVE   = "active";
    /** The key for the option array. */
    private static final String OPTIONS  = "options";
    /** The key for the instance number. */
    private static final String INSTANCE = ".instance";

    private static final AtomicInteger INSTANCE_COUNTER = new AtomicInteger();

    private final Map<String, T>        options;
    private final List<StringPublisher> activePublications = new ArrayList<>();
    private final ReentrantLock         mutex              = new ReentrantLock();
    private final int                   instanceNumber;

    private String defaultOptionName = "";
    private String selectedOption;

    public DTSendableChooser() {
        instanceNumber = INSTANCE_COUNTER.getAndIncrement();
        SendableRegistry.add(this, "DTSendableChooser", instanceNumber);

        options = new LinkedHashMap<>();
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
        mutex.lock();
        try {
            for (StringPublisher publisher : activePublications) {
                publisher.close();
            }
        } finally {
            mutex.unlock();
        }
    }

    public void addOption(String name, T value) {
        options.put(name, value);
    }

    public void setDefaultOption(String name, T value) {
        Objects.requireNonNull(name);

        addOption(name, value);
        defaultOptionName = name;
    }

    public T getSelected() {
        mutex.lock();
        try {
            return selectedOption == null ? options.get(defaultOptionName)
                    : options.get(selectedOption);
        } finally {
            mutex.unlock();
        }
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        IntegerPublisher instancePub = new IntegerTopic(builder.getTopic(INSTANCE)).publish();
        instancePub.set(instanceNumber);
        builder.addCloseable(instancePub);
        builder.addStringProperty(DEFAULT, () -> defaultOptionName, null);
        builder.addStringArrayProperty(OPTIONS, () -> options.keySet()
                                                             .toArray(String[]::new),
                null);
        builder.addStringProperty(ACTIVE, () -> {
            mutex.lock();
            try {
                return selectedOption == null ? defaultOptionName : selectedOption;
            } finally {
                mutex.unlock();
            }
        }, null);
        mutex.lock();
        try {
            activePublications.add(new StringTopic(builder.getTopic(ACTIVE)).publish());
        } finally {
            mutex.unlock();
        }
        builder.addStringProperty(SELECTED, null, (String value) -> {
            mutex.lock();
            try {
                selectedOption = value;
                for (StringPublisher publisher : activePublications) {
                    publisher.set(value);
                }
            } finally {
                mutex.unlock();
            }
        });
    }

    public void setSelection(String name) {
        mutex.lock();
        try {
            selectedOption = name;
        } finally {
            mutex.unlock();
        }
    }

    public void removeOption(String name) {
        mutex.lock();
        try {
            options.remove(name);
            if (name.equals(selectedOption)) {
                selectedOption = null;
            }
            if (name.equals(defaultOptionName)) {
                defaultOptionName = "";
            }
        } finally {
            mutex.unlock();
        }
    }

    public void removeAllOptions() {
        mutex.lock();
        try {
            options.clear();
            selectedOption = null;
            defaultOptionName = "";
        } finally {
            mutex.unlock();
        }
    }
}
