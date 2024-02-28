package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.Climber;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.drive.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.TwoWheelTrackingLocalizer;



@Autonomous(group="drive")
public class AutonomoAzul extends LinearOpMode {

    FieldOriented fieldOriented = new FieldOriented();

    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 0.35;
        fieldOriented.init(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Elevator arm = new Elevator(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        waitForStart();

        Waypoints[] waypoints = {
                new Waypoints(0, 10, 0, false, false, false, false, 50),
                new Waypoints(10, 0, 0, false, false, false, false, 50),
                new Waypoints(0, 0, 179, false, false, false, false,50),
                new Waypoints(0, 0, 0, false, false, false, false,50 )/*,
                new Waypoints(0, 0, 2000, false, false, true, false,500 ),
                new Waypoints(0, 0, 0, false, false, true, false,500 ),
                new Waypoints(0, 5, 0, false, false, true, false,0 ),
                new Waypoints(0, 0, 0, false, false, true, true,0 ),
                new Waypoints(0, 5, 0, false, false, false, false,0 ),*/
        };

        while (!isStopRequested()) {

            for (int idx=0; idx<waypoints.length;idx++) {
                if(isStopRequested()){
                    break;
                }
                Waypoints w =  waypoints[idx];
                fieldOriented.zeroEncoder();
                //fieldOriented.resetIMU();


                /*if(arm.getCurrentArmPosition()>10)
                    drive.setLimiterAuto(0.1);
                else
                    drive.setLimiterAuto(speed);*/
                if(w.x<0)
                    fieldOriented.fieldOrientedDrive(0, -speed, 0);

                else
                    fieldOriented.fieldOrientedDrive(0, speed, 0);


                while(Math.abs(fieldOriented.getPerpendicularPosition())<Math.abs( w.x)) {
                    updateTelemetry();
                }
                fieldOriented.fieldOrientedDrive(0, 0, 0);

                if(w.y<0)
                    fieldOriented.fieldOrientedDrive(-speed,0, 0);
                else
                    fieldOriented.fieldOrientedDrive(speed,0, 0);

                while(Math.abs(fieldOriented.getParallelPosition())<Math.abs( w.y)) {
                    updateTelemetry();
                }
                fieldOriented.fieldOrientedDrive(0, 0, 0);

                if(w.heading<0)
                    fieldOriented.fieldOrientedDrive(0,0,-speed);
                else
                    fieldOriented.fieldOrientedDrive(0,0,speed);

                while(Math.abs(fieldOriented.getRawExternalHeading())<Math.abs( w.heading)) {
                    updateTelemetry();
                }
                fieldOriented.fieldOrientedDrive(0, 0, 0);


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
                    arm.activate();
                }
                else{
                    arm.reverse();
                }
                if(w.openClaw){
                    claw.open();
                }
                else{
                    claw.close();
                }
                for (int i =0; i<(w.timeout/10);i++){
                    sleep(1);
                    arm.task(getRuntime());
                    updateTelemetry();
                }
            }

            break;
        }
        while (!isStopRequested()) {
            arm.task(getRuntime());
        }

    }
    void updateTelemetry(){
        telemetry.addData("Heading",fieldOriented.getRawExternalHeading());
        telemetry.addData("Parallel",fieldOriented.getParallelPosition());
        telemetry.addData("Perpendicular",fieldOriented.getPerpendicularPosition());

        telemetry.update();
    }

}
