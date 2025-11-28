package org.firstinspires.ftc.teamcode.common.menu;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.controller.Button;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;

import java.util.ArrayList;
import java.util.List;



public class Menu {

    private List<Screen> screens;
    private int currentScreen;
    private ButtonListener listener;
    private Telemetry telemetry;

    /**
     * Creates a menu from a listener, telemetry, and list of screens
     * @param listener a button listener
     * @param telemetry the telemetry
     * @param screens the list of screens
     */
    public Menu(ButtonListener listener, Telemetry telemetry, List<Screen> screens) {
        this.listener = listener;
        this.telemetry = telemetry;
        this.screens = screens;
        currentScreen = 0;

        listener.addListener(Button.D_UP_DOWN, this::scrollDown);
        listener.addListener(Button.D_DOWN_DOWN, this::scrollUp);
        listener.addListener(Button.CIRCLE_DOWN, this::back);
        listener.addListener(Button.CROSS_DOWN, this::select);
    }

    /**
     * Creates a menu from a listener, telemetry, and creates an empty list of screens
     * @param listener a button listener
     * @param telemetry the telemetry
     */
    public Menu(ButtonListener listener, Telemetry telemetry) {
        this(listener, telemetry, new ArrayList<>());
    }

    /**
     * Adds a screen to the menu in order
     * @param screen the screen to add
     * @return this
     */
    public Menu addScreen(Screen screen) {
        screens.add(screen);
        return this;
    }

    /**
     * Moves back a screen
     */
    private void back(){
        currentScreen = Math.max(currentScreen-1, 0);
    }

    /**
     * Moves the selection up on the current screen
     */
    private void scrollUp() {
        if (currentScreen < screens.size()) {
            screens.get(currentScreen).scrollUp();
        }
    }

    /**
     * Moves the selection down on the current screen
     */
    private void scrollDown() {
        if (currentScreen < screens.size()) {
            screens.get(currentScreen).scrollDown();
        }
    }

    /**
     * Displays the current view
     */
    private void display() {
        if (currentScreen < screens.size()) {
            telemetry.addLine(String.format("Select: %s", screens.get(currentScreen).toString()));
            telemetry.update();
        } else if (currentScreen == screens.size()){
            summary();
        }
    }

    /**
     * Checks if all screens have been completed
     * @return the competion status
     */
    public boolean isComplete() {
        return currentScreen > screens.size();
    }

    /**
     * Updates the listener and displays the content
     */
    public void update() {
        listener.update();
        display();

        if (isComplete()) {
            telemetry.addLine("Ready");
            telemetry.update();
        }
    }

    /**
     * Make a selection and move to the next screen
     */
    private void select() {
        currentScreen++;
    }

    /**
     * Displays a summary of the final selections
     * @return the summary
     */
    public Menu summary() {
        telemetry.addLine("Summary:");
        for (Screen screen : screens) {
            telemetry.addLine(screen.summary());
        }
        telemetry.addLine("Please Confirm");
        telemetry.update();
        return this;
    }
}