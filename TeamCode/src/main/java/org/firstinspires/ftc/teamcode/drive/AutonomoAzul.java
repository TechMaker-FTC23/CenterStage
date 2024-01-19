package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.Climber;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.TwoWheelTrackingLocalizer;



@Autonomous(group="drive")
public class AutonomoAzul extends LinearOpMode {

    TwoWheelTrackingLocalizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 0.3;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);
        Climber climber = new Climber(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        waitForStart();
        localizer = new TwoWheelTrackingLocalizer(hardwareMap,drive);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.resetElevatorEncoders();
        drive.resetHeading();
        drive.resetEncoder();
        Waypoints[] waypoints = {
                new Waypoints(0, 50, 0, false, false, false, false, 50),
                new Waypoints(0, 0, 0, false, false, false, false, 50),
                new Waypoints(0, 0, 0, false, false, false, false, 50),
                new Waypoints(0, 0, 0, false, false, true, false, 2500),
                new Waypoints(0,0,0,false,false,true,true,500),
                new Waypoints(0,0,0,false,false,false,false,2000),

        };

        while (!isStopRequested()) {

            for (int idx=0; idx<waypoints.length;idx++) {
                if(isStopRequested()){
                    break;
                }
                Waypoints w =  waypoints[idx];
                drive.resetEncoder();

                if(elevator.elevator_positionR>10)
                    drive.setLimiterAuto(0.1);
                else
                    drive.setLimiterAuto(0.4);
                if(w.x<0)
                    drive.setWeightedDrivePowerAuto(new Pose2d(-speed, 0, 0));
                else
                    drive.setWeightedDrivePowerAuto(new Pose2d(speed, 0, 0));

                while(Math.abs(localizer.getPerpendicularPosition())<Math.abs( w.x)) {
                    drive.update();
                    updateTelemetry();
                }

                if(w.y<0)
                    drive.setWeightedDrivePowerAuto(new Pose2d(0,-speed, 0));
                else
                    drive.setWeightedDrivePowerAuto(new Pose2d(0,speed, 0));

                while(Math.abs(localizer.getParallelPosition())<Math.abs( w.y)) {
                    drive.update();
                    updateTelemetry();
                }
                drive.setWeightedDrivePower(new Pose2d( 0, 0,speed));
                while(localizer.getHeading()>w.heading) {
                    updateTelemetry();
                }
                updateTelemetry();
                drive.setWeightedDrivePowerAuto(new Pose2d(0, 0, 0));
                if(w.actIntake){
                    intake.activate();
                }
                else if(w.actReverse){
                    intake.close();
                }
                else{
                    intake.stop();
                }
                if(w.extendElevator){
                    elevator.activate();
                }
                else{
                    elevator.reverse();
                }
                if(w.openClaw){
                    claw.open();
                }
                else{
                    claw.close();
                }
                for (int i =0; i<(w.timeout/10);i++){
                    sleep(1);
                    elevator.task();
                    updateTelemetry();
                }
            }

            break;
        }
        while (!isStopRequested()) {
            elevator.task();
        }

    }
    void updateTelemetry(){
        telemetry.addData("Heading",localizer.getHeading());
        telemetry.addData("Parallel",localizer.getParallelPosition());
        telemetry.addData("Perpendicular",localizer.getPerpendicularPosition());
        telemetry.update();
    }
}
