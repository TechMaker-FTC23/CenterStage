package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;


@TeleOp(group="drive")
public class testeTeleop extends LinearOpMode {



    public void runOpMode() {
        Elevator elevator = new Elevator(hardwareMap);

        waitForStart();
        while (!isStopRequested()) {

            if (gamepad1.square) {
                elevator.activate();
            } else if (gamepad1.cross) {
                elevator.reverse();
            }
            elevator.task(getRuntime());
        }
    }
}

