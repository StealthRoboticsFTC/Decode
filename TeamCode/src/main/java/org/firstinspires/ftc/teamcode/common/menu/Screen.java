package org.firstinspires.ftc.teamcode.common.menu;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents one "Screen" on the selection menu
 * @param <T> the type of data to select
 */
public class Screen<T> {
    private List<T> options;
    private String text;
    private int currentSelection;

    /**
     * Creates the screen with a title and the list of options
     * @param text the title to display
     * @param options the list of options
     */
    public Screen(String text, List<T> options) {
        this.options = options;
        this.text = text;
        currentSelection = 0;
    }

    /**
     * Creates the selection screen with just a title and no options
     * @param text the title text
     */
    public Screen(String text) {
        this(text, new ArrayList<>());
    }

    /**
     * Remove an option from the screen
     * @param option the option to remove
     * @return this
     */
    public Screen removeOption(T option) {
        options.remove(option);
        return this;
    }

    /**
     * Represents the final selection
     * @return formatted string of the selection
     */
    public String summary() {
        return String.format("%s: %s", text, options.get(currentSelection));
    }

    /**
     * An option to select
     * @param option
     * @return this
     */
    public Screen addOption(T option) {
        options.add(option);
        return this;
    }

    /**
     * Gets the selected option
     * @return selected option
     */
    public T getSelected() {
        return options.get(currentSelection);
    }

    /**
     * Moves the selection up
     */
    public void scrollUp() {
        currentSelection = Math.min(currentSelection + 1, options.size() - 1);
    }

    /**
     * Moves the selection down
     */
    public void scrollDown() {
        currentSelection = Math.max(currentSelection - 1, 0);
    }

    /**
     * String representation of the screen
     * @return the string representation of the screen
     */
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append(text);
        for (int i = 0; i < options.size(); i++) {
            builder.append('\n');
            builder.append(String.format(i == currentSelection ? ">%s<": "\u2002%s\u2002 ", options.get(i)));
        }
        return builder.toString();
    }
}
