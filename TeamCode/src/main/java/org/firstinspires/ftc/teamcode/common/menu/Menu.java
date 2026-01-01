package org.firstinspires.ftc.teamcode.common.menu;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.controller.Button;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;


public class Menu {

    private List<Screen> screens;
    private int currentScreen;
    private ButtonListener listener;
    private Telemetry telemetry;

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

    public Menu(ButtonListener listener, Telemetry telemetry) {
        this(listener, telemetry, new ArrayList<>());
    }

    public Menu addScreen(Screen screen) {
        screens.add(screen);
        return this;
    }

    private void back(){
        currentScreen = Math.max(currentScreen-1, 0);
    }

    private void scrollUp() {
        if (currentScreen < screens.size()) {
            screens.get(currentScreen).scrollUp();
        }
    }

    private void scrollDown() {
        if (currentScreen < screens.size()) {
            screens.get(currentScreen).scrollDown();
        }
    }

    private void display() {
        if (currentScreen < screens.size()) {
            telemetry.addLine(String.format("Select: %s", screens.get(currentScreen).toString()));
            telemetry.update();
        } else if (currentScreen == screens.size()){
            summary();
        }
    }

    public boolean isComplete() {
        return currentScreen > screens.size();
    }

    public void runBlocking() {
        while (!isComplete()) {
            listener.update();
            display();
        }
        telemetry.addLine("Ready");
        telemetry.update();
    }

    private void select() {
        currentScreen++;
    }


    public Menu summary() {
        telemetry.addLine("Summary:");
        for (Screen screen : screens) {
            telemetry.addLine(screen.summary());
        }
        telemetry.addLine("Please Confirm");
        telemetry.update();
        return this;
    }

    public List<Object> getSelections() {
        return screens.stream().map(Screen::getSelected).collect(Collectors.toList());
    }
}