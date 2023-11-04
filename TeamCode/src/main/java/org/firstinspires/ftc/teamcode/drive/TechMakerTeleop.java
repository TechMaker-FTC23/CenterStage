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
public class TechMakerTeleop extends LinearOpMode {
    public double mecanumVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake intake = new Intake(hardwareMap);

        Arm arm = new Arm(hardwareMap);

        Climber climber = new Climber(hardwareMap);

        Launcher launcher = new Launcher(hardwareMap);

        Claw claw = new Claw(hardwareMap);

        waitForStart();


        while (!isStopRequested()) {

            mecanumVelocity =1;// (5500 - arm.getCurrentArmPosition()) / 4500.0;
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y * mecanumVelocity, -gamepad1.left_stick_x * mecanumVelocity, -gamepad1.right_stick_x * mecanumVelocity));

            //Arm
            if(gamepad1.left_trigger>0.3){
                arm.activate();
            }
            if (gamepad1.right_trigger>0.3) {
                arm.reverse();
            }
            arm.task(telemetry);

            //Intake
            if (gamepad1.x){
                intake.activate();
            }
            else{
                intake.stop();
            }
            if (gamepad1.a){
                intake.reverse();
            }
            else{
                intake.stop();
            }

            //Climber
            if (gamepad1.y){
                climber.activate();
            }
            else {
                climber.stop();
            }
            //Claw
            if (gamepad1.right_bumper){
                claw.activate();
            }
            if (gamepad1.left_bumper){
                claw.reverse();
            }

            //Launcher
            if (gamepad1.b){
                launcher.activate();
            }

            //ResetEncoders
            if (gamepad1.back){
                arm.resetArmEncoders();
            }




        }

    }
}
