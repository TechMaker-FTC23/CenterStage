package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Led;
import org.firstinspires.ftc.teamcode.drive.subsystems.Climber;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.security.Timestamp;
import java.sql.Time;
import java.util.Timer;


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


        Led led = new Led(hardwareMap);

       Intake intake = new Intake(hardwareMap);

       Elevator elevator = new Elevator(hardwareMap);

       Launcher launcher = new Launcher(hardwareMap);

       Claw claw = new Claw(hardwareMap);


       elevator.resetElevatorEncoders();
        telemetry.addData("Posicao","Aguardando inicio");
        telemetry.update();
        waitForStart();

        claw.InitPosition();
        led.ligaLed();
        telemetry.addData("Posicao","Iniciado");
        telemetry.update();
        while (!isStopRequested()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            fieldOriented.fieldOrientedDrive(y, x, rx);
            telemetry.addData("Y",y);
            telemetry.addData("X",x);
            telemetry.addData("RX",rx);
            telemetry.addData("Tempo", getRuntime());
            telemetry.update();
            if (gamepad1.options) {
                fieldOriented.resetIMU();
            }

            /*
            mecanumVelocity = 1;
            drive.setWeightedDrivePower(

                  new Pose2d(-gamepad1.left_stick_y * mecanumVelocity, -gamepad1.left_stick_x * mecanumVelocity, -gamepad1.right_stick_x * mecanumVelocity));


             */


            //intake
            if (gamepad2.right_trigger > 0.5) {
                intake.activate();
            } else if(gamepad2.dpad_down) {intake.close();}
                else{intake.stop();
            }

            // Elevator

                if (gamepad2.square) {
                    elevator.activate();
                } else if (gamepad2.cross) {
                    elevator.reverse();
                }else if(gamepad2.circle){
                    elevator.climber();
                }else if(gamepad2.y){
                    elevator.desceClimber();
                }
                elevator.task(getRuntime());

            //elevator.task();

            //claw
            if (gamepad2.left_bumper) {
                claw.open();
                led.ligaLed();

            }else
            if (gamepad2.right_bumper) {
                claw.close();
                led.desligaLed();
            }

            //Launcher

            if(gamepad2.dpad_up){
                launcher.activate();
            }else {
                launcher.stop();
            }



        telemetry.addData("braço",elevator.getCurrentArmPosition());

        }
    }

}

