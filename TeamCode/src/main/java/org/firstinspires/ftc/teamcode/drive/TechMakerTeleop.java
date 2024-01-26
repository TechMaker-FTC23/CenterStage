package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.Climber;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(group="drive")
public class TechMakerTeleop extends LinearOpMode {
    public double mecanumVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //FieldOriented fieldOriented = new FieldOriented();
        //fieldOriented.init(hardwareMap);

        Intake intake = new Intake(hardwareMap);

        Elevator elevator = new Elevator(hardwareMap);

        //Climber climber = new Climber(hardwareMap);

        //Launcher launcher = new Launcher(hardwareMap);

        Claw claw = new Claw(hardwareMap);

        waitForStart();

        elevator.resetElevatorEncoders();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x));
            /*
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Call the fieldOrientedDrive method from the FieldOriented subsystem
            fieldOriented.fieldOrientedDrive(y, x, rx);

            if(gamepad1.options){
                fieldOriented.resetIMU();

             */
            }


            //Claw
            if(gamepad2.right_bumper){
                claw.close();}
            else if(gamepad2.left_trigger>50){
                claw.open();
            }

            // Elevator

          /*  while (gamepad2.left_trigger>0.5){
                elevator.activate();
            }
            while (gamepad2.right_trigger>0.5){
                elevator.reverse();
            }

           */

            //Launcher
            /*
            while (gamepad2.a){
                launcher.activate();
            }
            launcher.stop();


             */
            //intake + Claw
            if(gamepad2.right_bumper){
                intake.activate();
            }else{intake.stop();}
            /*
            //Climber
            if(gamepad2.a){
                climber.activate();
            }

             */

        }
    }

