package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;


@TeleOp(group="drive")
public class telemetryOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Elevator elevator = new Elevator(hardwareMap);

        waitForStart();
        elevator.resetElevatorEncoders();


        while (!isStopRequested()) {
            telemetry.addData("Left trigger",gamepad1.left_trigger);
            telemetry.addData("Right trigger",gamepad1.right_trigger);
           telemetry.addData("Encoder Braco",elevator.getCurrentArmPosition());
           telemetry.addData("esquerdo",elevator.left());
           telemetry.addData("direita",elevator.right());
           telemetry.update();


        }

    }
}
