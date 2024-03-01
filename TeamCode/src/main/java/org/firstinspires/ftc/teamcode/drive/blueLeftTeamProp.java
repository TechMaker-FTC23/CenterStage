package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;


@Autonomous(group="drive")
public class blueLeftTeamProp extends LinearOpMode {

    FieldOriented fieldOriented = new FieldOriented();

    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 0.4;
        double turnSpeed = 0.4;
        fieldOriented.init(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Elevator arm = new Elevator(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        Waypoints[] waypoints = {
                new Waypoints(0, 55, 0, false, false, false, false, false,100),
                new Waypoints(0,0, -76, false, false, false, false,false, 500),
                new Waypoints(0,0, 0, false, true, false, false,false, 500),
                new Waypoints(0,0, 76, false, false, false, false,false, 500),
                new Waypoints(0,10, 0, false, false, false, false,false, 50),


        };

        while (!isStopRequested()) {

            for (int idx=0; idx<waypoints.length;idx++) {
                if(isStopRequested()){
                    break;
                }
                Waypoints w =  waypoints[idx];
                fieldOriented.zeroEncoder();
                fieldOriented.resetIMU();


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
                    fieldOriented.fieldOrientedDrive(0,0,-turnSpeed);
                else
                    fieldOriented.fieldOrientedDrive(0,0,turnSpeed);

                while(Math.abs(fieldOriented.getRawExternalHeading())<Math.abs( w.heading)) {
                    updateTelemetry();
                }
                fieldOriented.fieldOrientedDrive(0, 0, 0);


                if(w.actIntake){
                    intake.activate();
                }
                else if(w.actReverse){
                    intake.reverseAutonomous();
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
