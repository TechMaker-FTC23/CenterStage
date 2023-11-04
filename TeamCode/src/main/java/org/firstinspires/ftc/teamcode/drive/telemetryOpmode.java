package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.Arm;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.Climber;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;


@TeleOp(group="drive")
public class telemetryOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Arm arm = new Arm(hardwareMap);

        waitForStart();
        arm.resetArmEncoders();


        while (!isStopRequested()) {
            telemetry.addData("Left trigger",gamepad1.left_trigger);
            telemetry.addData("Right trigger",gamepad1.right_trigger);
           telemetry.addData("Encoder Braco",arm.getCurrentArmPosition());
           telemetry.update();


        }

    }
}
