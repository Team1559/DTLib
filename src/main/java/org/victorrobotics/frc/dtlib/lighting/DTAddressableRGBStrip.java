package org.victorrobotics.frc.dtlib.lighting;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class DTAddressableRGBStrip {
    private final AddressableLED       leds;
    private final AddressableLEDBuffer buffer;
    private final int                  length;

    public DTAddressableRGBStrip(int port, int length) {
        this.length = length;
        buffer = new AddressableLEDBuffer(length);
        leds = new AddressableLED(port);
        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
    }

    public void setColor(Color color) {
        setColor(color.getRed(), color.getGreen(), color.getBlue());
    }

    public void setColor(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    public void setColor(Color color, int index) {
        setColor(color.getRed(), color.getGreen(), color.getBlue());
    }

    public void setColor(int r, int g, int b, int index) {
        buffer.setRGB(index, r, g, b);
    }
}
