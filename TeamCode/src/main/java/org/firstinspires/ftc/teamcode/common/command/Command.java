package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;

/**
 * A generic command that robot does (paths, subsystem movements, etc.)
 */
public interface Command {
    /**
     * Updates the command by one cycle
     * @param robot the robot
     */
    void update(Robot robot);

    /**
     * Checks if the task is complete
     * @return the status of completion of the task
     */
    boolean isDone();
}