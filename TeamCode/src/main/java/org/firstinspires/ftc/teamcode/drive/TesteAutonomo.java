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

@Autonomous(group="drive")
public class TesteAutonomo extends LinearOpMode {

    TwoWheelTrackingLocalizer localizer;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
        if (isStopRequested()) return;


        // Defina a trajetória 1
        Trajectory trajetoria1 = drive.trajectoryBuilder(new Pose2d(-72, 18, 0))
                .lineToSplineHeading(new Pose2d(-36, 40, Math.toRadians(90)))
                .build();

        // Siga a trajetória 1
        drive.followTrajectory(trajetoria1);



            // Permitir que outros processos do robô sejam executados
            drive.update();


        }
    }
}
