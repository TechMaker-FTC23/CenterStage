package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(group="drive")
public class TechMakerTeleop extends LinearOpMode {
    public double mecanumVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

       // drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FieldOriented fieldOriented = new FieldOriented();
        fieldOriented.runOpMode();

        Intake intake = new Intake(hardwareMap);

        Elevator elevator = new Elevator(hardwareMap);

        //Climber climber = new Climber(hardwareMap);

        Launcher launcher = new Launcher(hardwareMap);

        Claw claw = new Claw(hardwareMap);

        waitForStart();
       elevator.resetArmEncoders();



        while (!isStopRequested()) {
            /*mecanumVelocity = (5500 - 0) / 5600.0;
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y * mecanumVelocity, -gamepad1.left_stick_x * mecanumVelocity, -gamepad1.right_stick_x * mecanumVelocity));


             */
            // Arm
            while (gamepad2.left_trigger>0.5){
                elevator.teste();
            }
            while (gamepad2.right_trigger>0.5){
                elevator.voltaTeste();
            }


            //Launcher
            while (gamepad2.a){
                launcher.activate();
            }
            launcher.stop();

            //intake
            if(gamepad2.left_bumper){
                intake.activate();
                claw.open();
            }else if(gamepad2.right_bumper){
                intake.close();
            } else {
                intake.stop();
                claw.close();
            }

        }
    }
}
