package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.TwoWheelTrackingLocalizer;



@Autonomous(group="drive")
public class AutonomoAzul extends LinearOpMode {

    TwoWheelTrackingLocalizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        waitForStart();
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launcher.stop();
        //elevator.resetElevatorEncoders();
        drive.resetHeading();
        drive.resetEncoder();
        Waypoints[] waypoints = {
                /*new Waypoints(-36, 0, 0, 200),
                new Waypoints(0, 30, 0, 200)*/
                new Waypoints(-10, 0, 0,0,0,0 )

        };

        while (!isStopRequested()) {
            double speed = 0.5;

            for (int idx = 0; idx < waypoints.length; idx++) {
                if (isStopRequested()) {
                    break;
                }
                Waypoints w = waypoints[idx];
                drive.resetEncoder();

                drive.setLimiterAuto(0.01);
                if (w.right < 0)
                    drive.setWeightedDrivePowerAuto(new Pose2d(-speed, 0, 0));
                else
                    drive.setWeightedDrivePowerAuto(new Pose2d(speed, 0, 0));

                while (Math.abs(localizer.getPerpendicularPosition()) < Math.abs(w.right)) {
                    drive.update();
                    updateTelemetry(w);
                }



                if (w.left < 0)
                    drive.setWeightedDrivePowerAuto(new Pose2d(0, -speed, 0));
                else
                    drive.setWeightedDrivePowerAuto(new Pose2d(0, speed, 0));

                while (Math.abs(localizer.getParallelPosition()) < Math.abs(w.left)) {
                    drive.update();
                    updateTelemetry(w);
                }

                while (localizer.getHeading() > w.heading) {
                    drive.setWeightedDrivePowerAuto(new Pose2d(0, 0, speed));
                    updateTelemetry(w);
                }
                updateTelemetry(w);
                drive.setWeightedDrivePowerAuto(new Pose2d(0, 0, 0));
                for (int i =0; i<w.timeout;i++){
                    sleep(1);
                    //elevator.task();
                    updateTelemetry(w);
                }


            }

            break;
        }

    }
    void updateTelemetry(Waypoints w){
        telemetry.addData("Heading",localizer.getHeading());
        telemetry.addData("Parallel",localizer.getParallelPosition());
        telemetry.addData("Perpendicular",localizer.getPerpendicularPosition());
        telemetry.addData("Tempo",w.timeout);
        telemetry.addData("Waypoint X",w.right);
        telemetry.addData("Waypoint Y",w.left);
        telemetry.addData("Waypoint H",w.heading);
        telemetry.update();
    }
}
