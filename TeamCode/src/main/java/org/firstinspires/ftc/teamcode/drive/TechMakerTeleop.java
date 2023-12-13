package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;


@TeleOp(group="drive")
public class TechMakerTeleop extends LinearOpMode {
    public double mecanumVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Intake intake = new Intake(hardwareMap);

        Elevator elevator = new Elevator(hardwareMap);

        //Climber climber = new Climber(hardwareMap);

        //Launcher launcher = new Launcher(hardwareMap);

        Claw claw = new Claw(hardwareMap);

        waitForStart();
        elevator.resetArmEncoders();



        while (!isStopRequested()) {

            /*
            mecanumVelocity = (5500 - arm.getCurrentArmPosition()) / 5600.0;
            drive.setWeightedDrivePower(
                    new Pose2d(-gamepad1.left_stick_y * mecanumVelocity, -gamepad1.left_stick_x * mecanumVelocity, -gamepad1.right_stick_x * mecanumVelocity));
             */
            //Arm
            if(gamepad1.left_trigger>0.75){
                elevator.teste();
            }
            if (gamepad1.right_trigger>0.73) {
                elevator.voltaTeste();
            }
           //arm.task();

            // claw
            if (gamepad1.right_bumper) {
                claw.open();
            }
            else if(gamepad1.left_bumper){
                claw.close();
            }

            /*
            //Launcher
            if (gamepad1.b){
                launcher.activate();
            }else{launcher.stop();}

            //ResetEncoders
            if (gamepad1.back){
                arm.resetArmEncoders();
            }
             */
        }
    }
}
