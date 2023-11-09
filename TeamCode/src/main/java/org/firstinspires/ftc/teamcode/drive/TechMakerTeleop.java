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
        arm.resetArmEncoders();


        while (!isStopRequested()) {

            mecanumVelocity = (5500 - arm.getCurrentArmPosition()) / 5600.0;
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y * mecanumVelocity, -gamepad1.left_stick_x * mecanumVelocity, -gamepad1.right_stick_x * mecanumVelocity));



            //Arm
            if(gamepad1.left_trigger>0.75){
                arm.activate();
            }
            if (gamepad1.right_trigger>0.73) {
                arm.reverse();
            }
            arm.task();

            //Intake  claw
            if (gamepad1.right_bumper){
                claw.open();
                intake.activate();
            }
            else if (gamepad1.a){
                intake.close();
            }
            else{
                claw.close();
                intake.stop();

            }

            if (gamepad1.left_bumper){
                claw.open();
                sleep(500);
            }
            //Climber
            if (gamepad1.y){
                climber.activate();
            }
            else if(gamepad1.dpad_up) {
                climber.reverse();
            }else{
                climber.stop();
            }


            //Launcher
            if (gamepad1.b){
                launcher.activate();
            }else{launcher.stop();}

            //ResetEncoders
            if (gamepad1.back){
                arm.resetArmEncoders();
            }
        }

    }
}
