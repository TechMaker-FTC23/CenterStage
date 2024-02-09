package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Climber;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(group="drive")
public class TechMakerTeleop extends LinearOpMode {
    public double mecanumVelocity = 0;
    /*
     0 - Em baixo
     1 - Subindo
     2 - Em cima
     3 - Descendo
     */
    public int elevatorState = 0;

    @Override
    public void runOpMode() {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FieldOriented fieldOriented = new FieldOriented();
        fieldOriented.init(hardwareMap);

       // Intake intake = new Intake(hardwareMap);

       // Elevator elevator = new Elevator(hardwareMap);

        // Climber climber = new Climber(hardwareMap);

        Launcher launcher = new Launcher(hardwareMap);

        Claw claw = new Claw(hardwareMap);

        claw.open();
       // elevator.resetElevatorEncoders();
        waitForStart();
        while (!isStopRequested()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            fieldOriented.fieldOrientedDrive(y, x, rx);

            if (gamepad1.options) {
                fieldOriented.resetIMU();
            }
           // mecanumVelocity = 1;
          //  drive.setWeightedDrivePower(
          //
            //          new Pose2d(-gamepad1.left_stick_y * mecanumVelocity, -gamepad1.left_stick_x * mecanumVelocity, -gamepad1.right_stick_x * mecanumVelocity));

            /*
            //intake
            if (gamepad2.right_trigger > 0.5) {
                intake.activate();
            } else if(gamepad2.dpad_down) {intake.close();}
                else{intake.stop();
            }

            // Elevator
            if (gamepad2.square) {
                elevatorState = 1;

            }
            if (gamepad2.cross) {
                claw.open();
                elevatorState = 3;

            }

            if (elevatorState == 1) {
                if (elevator.finalPosition()) {
                    elevatorState = 2;
                }
            }
            if (elevatorState == 3) {
                if (elevator.initialPosition()) {
                    elevatorState = 0;
                }
            }
            elevator.clawPosition();

            elevator.task();

            //claw
            if (gamepad2.left_bumper) {
                claw.openLeft();
            }
            if (gamepad2.right_bumper) {
                claw.openRight();
            } else if (gamepad2.left_trigger > 0.5) {
                claw.close();
            }

            if (gamepad2.dpad_up) {
                launcher.activate();
            } else {
                launcher.stop();
            }
             */
        }
    }
}

