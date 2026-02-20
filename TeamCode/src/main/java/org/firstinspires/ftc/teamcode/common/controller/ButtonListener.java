package org.firstinspires.ftc.teamcode.common.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * A listener for the buttons on the gamepad
 */
public class ButtonListener {
    private final Gamepad last;
    private final Gamepad current;
    private final Gamepad gamepad;
    private final HashMap<Button, ArrayList<Runnable>> listeners;

    /**
     * Crates the listener from a gamepad
     * @param gamepad the gamepad to listen
     */
    public ButtonListener(Gamepad gamepad) {
        last = new Gamepad();
        current = new Gamepad();
        this.gamepad = gamepad;
        listeners = new HashMap<>();
    }

    /**
     * Updates the listener by checking the gamepad
     * @return this
     */
    public ButtonListener update() {
        last.copy(current);
        current.copy(gamepad);

        if (current.triangle && !last.triangle) {
            triggerListeners(Button.TRIANGLE_DOWN);
        }
        if (current.circle && !last.circle) {
            triggerListeners(Button.CIRCLE_DOWN);
        }
        if (current.cross && !last.cross) {
            triggerListeners(Button.CROSS_DOWN);
        }
        if (current.square && !last.square) {
            triggerListeners(Button.SQUARE_DOWN);
        }
        if (current.left_trigger > 0.5 && last.left_trigger < 0.5) {
            triggerListeners(Button.L_TRIGGER_DOWN);
        }
        if (current.right_trigger > 0.5 && last.right_trigger < 0.5) {
            triggerListeners(Button.R_TRIGGER_DOWN);
        }
        if (current.left_bumper && !last.left_bumper) {
            triggerListeners(Button.L_BUMPER_DOWN);
        }
        if (current.right_bumper && !last.right_bumper) {
            triggerListeners(Button.R_BUMPER_DOWN);
        }
        if (current.dpad_up && !last.dpad_up) {
            triggerListeners(Button.D_UP_DOWN);
        }
        if (current.dpad_right && !last.dpad_right) {
            triggerListeners(Button.D_RIGHT_DOWN);
        }
        if (current.dpad_down && !last.dpad_down) {
            triggerListeners(Button.D_DOWN_DOWN);
        }
        if (current.dpad_left && !last.dpad_left) {
            triggerListeners(Button.D_LEFT_DOWN);
        }
        if (current.touchpad && !last.touchpad) {
            triggerListeners(Button.TOUCHPAD_DOWN);
        }
        if (current.left_stick_button && !last.left_stick_button) {
            triggerListeners(Button.L_STICK_DOWN);
        }
        if (current.right_stick_button && !last.right_stick_button) {
            triggerListeners(Button.R_STICK_DOWN);
        }



        if (!current.triangle && last.triangle) {
            triggerListeners(Button.TRIANGLE_UP);
        }
        if (!current.circle && last.circle) {
            triggerListeners(Button.CIRCLE_UP);
        }
        if (!current.cross && last.cross) {
            triggerListeners(Button.CROSS_UP);
        }
        if (!current.square && last.square) {
            triggerListeners(Button.SQUARE_UP);
        }
        if (!current.left_bumper && last.left_bumper) {
            triggerListeners(Button.L_BUMPER_UP);
        }
        if (!current.right_bumper && last.right_bumper) {
            triggerListeners(Button.R_BUMPER_UP);
        }
        if (!current.dpad_up && last.dpad_up) {
            triggerListeners(Button.D_UP_UP);
        }
        if (!current.dpad_right && last.dpad_right) {
            triggerListeners(Button.D_RIGHT_UP);
        }
        if (!current.dpad_down && last.dpad_down) {
            triggerListeners(Button.D_DOWN_UP);
        }
        if (!current.dpad_left && last.dpad_left) {
            triggerListeners(Button.D_LEFT_UP);
        }
        if (!current.touchpad && last.touchpad) {
            triggerListeners(Button.TOUCHPAD_UP);
        }
        if (!current.left_stick_button && last.left_stick_button) {
            triggerListeners(Button.L_STICK_UP);
        }
        if (!current.right_stick_button && last.right_stick_button) {
            triggerListeners(Button.R_STICK_UP);
        }

        return this;
    }

    /**
     * Calls all the listeners with updates
     * @param button the update
     */
    private void triggerListeners(Button button) {
        if (listeners.containsKey(button)) {
            for (Runnable listener : listeners.get(button)) {
                listener.run();
            }
        }
    }

    /**
     * Adds a new listener for a certain button
     * @param button the button to listen for
     * @param listener the callback
     * @return this
     */
    public ButtonListener addListener(Button button, Runnable listener) {
        if (!listeners.containsKey(button)) {
            listeners.put(button, new ArrayList<>());
        }
        listeners.get(button).add(listener);
        return this;
    }

    /**
     * Removes the listeners
     * @return this
     */
    public ButtonListener clearListeners() {
        listeners.clear();
        return this;
    }
}
