package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Captures what the OpMode would show on the Driver Station. */
public class SimTelemetry implements Telemetry {
    private final Map<String, String> pending = new LinkedHashMap<>();
    private final List<String> lines = new ArrayList<>();
    private Map<String, String> shown = new LinkedHashMap<>();
    private boolean autoClear = true;

    public Map<String, String> shown() {
        return shown;
    }

    public String get(String caption) {
        return shown.get(caption);
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        pending.put(caption, String.format(format, args));
        return null;
    }

    @Override
    public Item addData(String caption, Object value) {
        pending.put(caption, String.valueOf(value));
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        pending.put(caption, String.valueOf(valueProducer.value()));
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        pending.put(caption, String.format(format, valueProducer.value()));
        return null;
    }

    @Override public boolean removeItem(Item item) { return false; }
    @Override public void clear() { pending.clear(); lines.clear(); }
    @Override public void clearAll() { clear(); }
    @Override public Object addAction(Runnable action) { return null; }
    @Override public boolean removeAction(Object token) { return false; }
    @Override public void speak(String text) { }
    @Override public void speak(String text, String languageCode, String countryCode) { }

    @Override
    public boolean update() {
        shown = new LinkedHashMap<>(pending);
        if (autoClear) {
            pending.clear();
            lines.clear();
        }
        return true;
    }

    @Override public Line addLine() { return null; }
    @Override public Line addLine(String lineCaption) { lines.add(lineCaption); return null; }
    @Override public boolean removeLine(Line line) { return false; }
    @Override public boolean isAutoClear() { return autoClear; }
    @Override public void setAutoClear(boolean autoClear) { this.autoClear = autoClear; }
    @Override public int getMsTransmissionInterval() { return 250; }
    @Override public void setMsTransmissionInterval(int ms) { }
    @Override public String getItemSeparator() { return " | "; }
    @Override public void setItemSeparator(String s) { }
    @Override public String getCaptionValueSeparator() { return " : "; }
    @Override public void setCaptionValueSeparator(String s) { }
    @Override public void setDisplayFormat(DisplayFormat displayFormat) { }
    @Override public Log log() { return null; }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, String> e : shown.entrySet()) {
            sb.append(e.getKey()).append(" : ").append(e.getValue()).append('\n');
        }
        return sb.toString();
    }
}
