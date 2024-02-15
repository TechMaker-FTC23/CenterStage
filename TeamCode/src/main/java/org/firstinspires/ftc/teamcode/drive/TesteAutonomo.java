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
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        waitForStart();
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.resetHeading();
        drive.resetEncoder();
        Waypoints[] waypoints = {
                new Waypoints(0, 0, 0, 0,0,0),
                new Waypoints(0, 0, 0, 0,0,0),
                new Waypoints(0, 0, 0, 0,0,0)
        };

        while (!isStopRequested()) {
            double speed = 0.5;

            for (int idx = 0; idx < waypoints.length; idx++) {
                if (isStopRequested()) {
                    break;
                }
                Waypoints w = waypoints[idx];
                drive.resetEncoder();

                drive.setLimiterAuto(0.5);

                // Constrói a trajetória para frente usando o valor de forward fornecido no waypoint
                TrajectoryBuilder trajetoriaFrenteBuilder = drive.trajectoryBuilder(new Pose2d())
                        .forward(w.foward);

                Trajectory trajetoriaFrente = trajetoriaFrenteBuilder.build();

                if (w.foward > 0) {
                    drive.followTrajectory(trajetoriaFrente);
                }

                while (Math.abs(localizer.getPerpendicularPosition()) < Math.abs(w.foward)) {
                    drive.update();
                    updateTelemetry(w);
                }

                // Constrói a trajetória para trás usando o valor de back fornecido no waypoint
                TrajectoryBuilder trajetoriaTrasBuilder = drive.trajectoryBuilder(new Pose2d())
                        .back(w.back);

                Trajectory trajetoriaTras = trajetoriaTrasBuilder.build();

                if (w.back > 0) {
                    drive.followTrajectory(trajetoriaTras);
                }

                TrajectoryBuilder direitaBuilder = drive.trajectoryBuilder(trajetoriaTras.end())
                        .strafeLeft(speed);

                Trajectory trajetoriaDireita = direitaBuilder.build();

                if(w.right>0) {
                    drive.followTrajectory(trajetoriaDireita);
                }

                TrajectoryBuilder diagonal = drive.trajectoryBuilder(new Pose2d())
                        .lineToSplineHeading(new Pose2d(speed, speed, speed));

                Trajectory trajetoriaDiagonal = diagonal.build();

                if(w.right>0 && w.foward>0 && w.heading>0){
                    drive.followTrajectory(trajetoriaDiagonal);
                }




                while (Math.abs(localizer.getParallelPosition()) < Math.abs(w.back)) {
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
        telemetry.addData("Waypoint X",w.foward);
        telemetry.addData("Waypoint Y",w.back);
        telemetry.addData("Waypoint H",w.heading);
        telemetry.update();
    }
}
