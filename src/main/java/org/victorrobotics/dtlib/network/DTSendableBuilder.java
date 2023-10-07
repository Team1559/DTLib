package org.victorrobotics.dtlib.network;

import org.victorrobotics.dtlib.function.supplier.DTConditionalBooleanSupplier;
import org.victorrobotics.dtlib.function.supplier.DTConditionalDoubleSupplier;
import org.victorrobotics.dtlib.function.supplier.DTConditionalFloatSupplier;
import org.victorrobotics.dtlib.function.supplier.DTConditionalLongSupplier;
import org.victorrobotics.dtlib.function.supplier.DTConditionalSupplier;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanArrayTopic;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.RawTopic;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;

public class DTSendableBuilder implements NTSendableBuilder {
  @FunctionalInterface
  private interface TimestampedConsumer<T> {
    void accept(T value, long timeMicros);
  }

  private static class Property<P extends Publisher, S extends Subscriber>
      implements AutoCloseable {
    P publisher;
    S subscriber;

    TimestampedConsumer<P> publisherCallback;
    Consumer<S>            subscriberCallback;

    @Override
    public void close() {
      try {
        if (publisher != null) {
          publisher.close();
        }
      } catch (Exception e) {
        // TODO: handle exception
      }
      try {
        if (subscriber != null) {
          subscriber.close();
        }
      } catch (Exception e) {
        // TODO: handle exception
      }
    }

    void update(boolean listening, long time) {
      if (listening && subscriber != null && subscriberCallback != null) {
        subscriberCallback.accept(subscriber);
      }
      if (publisher != null && publisherCallback != null) {
        publisherCallback.accept(publisher, time);
      }
    }
  }

  private final List<Property>      properties   = new ArrayList<>();
  private final List<Runnable>      updateTables = new ArrayList<>();
  private final List<AutoCloseable> closeables   = new ArrayList<>();

  private Runnable     makeSafe;
  private NetworkTable table;
  private boolean      isListening;
  private boolean      isActuator;

  private BooleanPublisher listeningPub;
  private BooleanPublisher actuatorPub;
  private StringPublisher  typePub;

  @Override
  public void setSmartDashboardType(String type) {
    if (typePub == null) {
      typePub = table.getStringTopic(".type")
                     .publish();
    }
    typePub.set(type);
  }

  @Override
  public void setActuator(boolean value) {
    if (actuatorPub == null) {
      actuatorPub = table.getBooleanTopic(".actuator")
                         .publish();
    }
    actuatorPub.set(value);
    isActuator = value;
  }

  @Override
  public void setSafeState(Runnable func) {
    makeSafe = func;
  }

  @Override
  public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
    Property<BooleanPublisher, BooleanSubscriber> property = new Property<>();
    BooleanTopic topic = table.getBooleanTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalLongSupplier) {
        DTConditionalBooleanSupplier conditionalGetter = (DTConditionalBooleanSupplier) getter;
        property.publisherCallback = (BooleanPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.getAsBoolean(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.getAsBoolean(), time);
      }
    }

    if (setter != null) {
      property.subscriber =
          topic.subscribe(false, PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (BooleanSubscriber sub) -> {
        for (boolean val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addIntegerProperty(String key, LongSupplier getter, LongConsumer setter) {
    Property<IntegerPublisher, IntegerSubscriber> property = new Property<>();
    IntegerTopic topic = table.getIntegerTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalLongSupplier) {
        DTConditionalLongSupplier conditionalGetter = (DTConditionalLongSupplier) getter;
        property.publisherCallback = (IntegerPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.getAsLong(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.getAsLong(), time);
      }
    }

    if (setter != null) {
      property.subscriber = topic.subscribe(0, PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (IntegerSubscriber sub) -> {
        for (long val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addFloatProperty(String key, FloatSupplier getter, FloatConsumer setter) {
    Property<FloatPublisher, FloatSubscriber> property = new Property<>();
    FloatTopic topic = table.getFloatTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalFloatSupplier) {
        DTConditionalFloatSupplier conditionalGetter = (DTConditionalFloatSupplier) getter;
        property.publisherCallback = (FloatPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.getAsFloat(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.getAsFloat(), time);
      }
    }

    if (setter != null) {
      property.subscriber = topic.subscribe(0, PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (FloatSubscriber sub) -> {
        for (float val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
    Property<DoublePublisher, DoubleSubscriber> property = new Property<>();
    DoubleTopic topic = table.getDoubleTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalDoubleSupplier) {
        DTConditionalDoubleSupplier conditionalGetter = (DTConditionalDoubleSupplier) getter;
        property.publisherCallback = (DoublePublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.getAsDouble(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.getAsDouble(), time);
      }
    }

    if (setter != null) {
      property.subscriber = topic.subscribe(0, PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (DoubleSubscriber sub) -> {
        for (double val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
    Property<StringPublisher, StringSubscriber> property = new Property<>();
    StringTopic topic = table.getStringTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<String> conditionalGetter = (DTConditionalSupplier<String>) getter;
        property.publisherCallback = (StringPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber = topic.subscribe("", PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (StringSubscriber sub) -> {
        for (String val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter,
                                      Consumer<boolean[]> setter) {
    Property<BooleanArrayPublisher, BooleanArraySubscriber> property = new Property<>();
    BooleanArrayTopic topic = table.getBooleanArrayTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<boolean[]> conditionalGetter =
            (DTConditionalSupplier<boolean[]>) getter;
        property.publisherCallback = (BooleanArrayPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber =
          topic.subscribe(new boolean[0], PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (BooleanArraySubscriber sub) -> {
        for (boolean[] val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addIntegerArrayProperty(String key, Supplier<long[]> getter,
                                      Consumer<long[]> setter) {
    Property<IntegerArrayPublisher, IntegerArraySubscriber> property = new Property<>();
    IntegerArrayTopic topic = table.getIntegerArrayTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<long[]> conditionalGetter = (DTConditionalSupplier<long[]>) getter;
        property.publisherCallback = (IntegerArrayPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber =
          topic.subscribe(new long[0], PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (IntegerArraySubscriber sub) -> {
        for (long[] val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addFloatArrayProperty(String key, Supplier<float[]> getter,
                                    Consumer<float[]> setter) {
    Property<FloatArrayPublisher, FloatArraySubscriber> property = new Property<>();
    FloatArrayTopic topic = table.getFloatArrayTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<float[]> conditionalGetter = (DTConditionalSupplier<float[]>) getter;
        property.publisherCallback = (FloatArrayPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber =
          topic.subscribe(new float[0], PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (FloatArraySubscriber sub) -> {
        for (float[] val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addDoubleArrayProperty(String key, Supplier<double[]> getter,
                                     Consumer<double[]> setter) {
    Property<DoubleArrayPublisher, DoubleArraySubscriber> property = new Property<>();
    DoubleArrayTopic topic = table.getDoubleArrayTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<double[]> conditionalGetter =
            (DTConditionalSupplier<double[]>) getter;
        property.publisherCallback = (DoubleArrayPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber =
          topic.subscribe(new double[0], PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (DoubleArraySubscriber sub) -> {
        for (double[] val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addStringArrayProperty(String key, Supplier<String[]> getter,
                                     Consumer<String[]> setter) {
    Property<StringArrayPublisher, StringArraySubscriber> property = new Property<>();
    StringArrayTopic topic = table.getStringArrayTopic(key);

    if (getter != null) {
      property.publisher = topic.publish();
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<String[]> conditionalGetter =
            (DTConditionalSupplier<String[]>) getter;
        property.publisherCallback = (StringArrayPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber =
          topic.subscribe(new String[0], PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (StringArraySubscriber sub) -> {
        for (String[] val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public void addRawProperty(String key, String typeString, Supplier<byte[]> getter,
                             Consumer<byte[]> setter) {
    Property<RawPublisher, RawSubscriber> property = new Property<>();
    RawTopic topic = table.getRawTopic(key);

    if (getter != null) {
      property.publisher = topic.publish(typeString);
      if (getter instanceof DTConditionalSupplier) {
        DTConditionalSupplier<byte[]> conditionalGetter = (DTConditionalSupplier<byte[]>) getter;
        property.publisherCallback = (RawPublisher pub, long time) -> {
          if (conditionalGetter.update()) {
            pub.set(conditionalGetter.get(), time);
          }
        };
      } else {
        property.publisherCallback = (pub, time) -> pub.set(getter.get(), time);
      }
    }

    if (setter != null) {
      property.subscriber = topic.subscribe(typeString, new byte[0],
                                            PubSubOption.excludePublisher(property.publisher));
      property.subscriberCallback = (RawSubscriber sub) -> {
        for (byte[] val : sub.readQueueValues()) {
          setter.accept(val);
        }
      };
    }

    properties.add(property);
  }

  @Override
  public boolean isPublished() {
    return table != null;
  }

  @Override
  public void update() {
    long timeMicros = WPIUtilJNI.now();
    for (Property property : properties) {
      property.update(isListening, timeMicros);
    }
    for (Runnable updateTable : updateTables) {
      updateTable.run();
    }
  }

  @Override
  public void clearProperties() {
    // stop listeners
    for (Property property : properties) {
      property.close();
    }
    properties.clear();
  }

  @Override
  public void addCloseable(AutoCloseable closeable) {
    closeables.add(closeable);
  }

  @Override
  public void close() throws Exception {
    if (listeningPub != null) {
      listeningPub.close();
    }
    if (typePub != null) {
      typePub.close();
    }
    if (actuatorPub != null) {
      actuatorPub.close();
    }
    for (Property property : properties) {
      property.close();
    }
    for (AutoCloseable closeable : closeables) {
      try {
        closeable.close();
      } catch (Exception e) {
        // ignore
      }
    }
  }

  @Override
  public void setUpdateTable(Runnable func) {
    updateTables.add(func);
  }

  @Override
  public Topic getTopic(String key) {
    return table.getTopic(key);
  }

  @Override
  public NetworkTable getTable() {
    return table;
  }

  public void setTable(NetworkTable table) {
    this.table = table;
    listeningPub = table.getBooleanTopic(".controllable")
                        .publish();
    listeningPub.setDefault(false);
  }

  public boolean isActuator() {
    return isActuator;
  }

  public void startListeners() {
    isListening = true;
    if (listeningPub != null) {
      listeningPub.set(true);
    }
  }

  public void stopListeners() {
    isListening = false;
    if (listeningPub != null) {
      listeningPub.set(false);
    }
  }

  public void startLiveWindowMode() {
    if (makeSafe != null) {
      makeSafe.run();
    }
    startListeners();
  }

  public void stopLiveWindowMode() {
    stopListeners();
    if (makeSafe != null) {
      makeSafe.run();
    }
  }
}
