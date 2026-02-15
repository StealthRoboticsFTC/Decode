package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.command.Command;

import java.util.Deque;
import java.util.LinkedList;

/**
 * Creates an async queue to execute your commands.
 * You can just set the current command or also manipulate the queue to schedule commands.
 */
public class Processor {

    private final Deque<Command> commandQueue;
    private long lastTime;

    Command lastExecuted;

    /**
     * constructs an empty processor
     */
    public Processor() {
        commandQueue = new LinkedList<>();
        lastTime = System.currentTimeMillis();
        lastExecuted = null;
    }

    /**
     * Check the status of the current command, pulls a new command from the queue if needed.
     * Runs the update of the current command one cycle.
     *
     * @param robot the robot
     */
    public void update(Robot robot) {
        long dt = System.currentTimeMillis() - lastTime;
        if (!commandQueue.isEmpty()) {
            Command current = commandQueue.peek();
            if (current == null || current.isDone()) {
                lastExecuted = commandQueue.remove();
            }

            if (current != null) {
                current.update(robot);
            }
        }
        lastTime = System.currentTimeMillis();
    }

    /**
     * Checks if the queue has tasks
     *
     * @return if the queue has work to do
     */
    public boolean isBusy() {
        return !commandQueue.isEmpty();
    }

    /**
     * Add a new Command to the processor
     *
     * @param command the command to be added
     */
    public void add(Command command) {
        commandQueue.add(command);
    }

    /**
     * Gets the last command in the queue
     *
     * @return the last command in the queue
     */


    /**
     * Clears the queue and adds the current action
     *
     * @param command
     */
    public Command override(Command command) {
        commandQueue.clear();
        add(command);
        return command;
    }
    public Command getLastExecuted(){
        return lastExecuted;
    }

    public Command getCommand(){
        return commandQueue.peek();
    }
}
