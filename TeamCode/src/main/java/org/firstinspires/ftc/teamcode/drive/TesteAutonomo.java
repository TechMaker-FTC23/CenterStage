package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.TwoWheelTrackingLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;

@Autonomous(group="drive")
public class TesteAutonomo extends LinearOpMode {

    TwoWheelTrackingLocalizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Intake intake = new Intake(hardwareMap);
        Trajectory trajetoria1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(18)
                .build();

        Trajectory trajetoria2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(7)
                        .build();

        Trajectory trajetoria3 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(5,5,90))
                        .build();


        /*


     */

        waitForStart();

        drive.followTrajectory(trajetoria3);


        while (opModeIsActive() && !isStopRequested()) {
            // Permitir que outros processos do robô sejam executados
            drive.update();
            idle();
        }
    }
}
