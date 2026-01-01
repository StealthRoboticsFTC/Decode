package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.menu.Menu;
import org.firstinspires.ftc.teamcode.common.menu.Screen;

import java.util.Arrays;
import java.util.List;
@TeleOp

public class menuTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Menu menu = new Menu(new ButtonListener(gamepad1), telemetry);
        menu.addScreen(new Screen("Color", Arrays.asList(Color.values())));

        menu.runBlocking();

        List<Object> options = menu.getSelections();
        Robot.color = (Color) options.get(0);

        waitForStart();

        while (!isStopRequested()){
            telemetry.addData("Color", Robot.color);
            telemetry.update();
        }

    }
}
