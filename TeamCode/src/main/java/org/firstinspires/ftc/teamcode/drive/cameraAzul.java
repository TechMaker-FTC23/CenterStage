package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.subsystems.Claw;
import org.firstinspires.ftc.teamcode.drive.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.subsystems.FieldOriented;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Launcher;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class cameraAzul extends LinearOpMode {
    FieldOriented fieldOriented = new FieldOriented();
    int direction = 0;// 0 - Meio, 1 - direita, 2 - esquerda

    OpenCvWebcam webcam = null; //declarar a webcam
    String linha;
    @Override
    public void runOpMode() {



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //mapeamento da webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); //declaração do id da webcam

        webcam.setPipeline(new Pipeline()); //definindo o pipeline

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT); //definindo o tamanho de tela da sua câmera
            } //logitech c920 fullHD

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();


        Waypoints[] waypoints = null;

        while (!isStopRequested()) {
            if(direction==1)
            {
                Waypoints[] wp1 = {
                        new Waypoints(0,0,0,false,false,false,false,false,50),
                };
                executeWaypoints(wp1);

                //movimentação pela direita
            }
            else if(direction==2){

                Waypoints[] wp2 = {
                        new Waypoints(0, 10, 0, false, false, false, false,false, 50),
                        new Waypoints(-30, 0, 0, false, false, false, false,false, 500),
                        new Waypoints(0,0,-84,false,false,false,false,false,50),
                        new Waypoints(0,0,0,false,true,false,false,false,50),
                };
                executeWaypoints(wp2);

                //movimentação pela esquerda
            }
            else{

                Waypoints[] wp3 = {
                        new Waypoints(0, 0, 0, false, false, false, false,false, 0),
                        new Waypoints(0, 0, 0, false, true, false, false,false, 0),
                        new Waypoints(0, 0, 0, false, false, false, false,false, 0),
                        new Waypoints(0, 0, 0, false, false, false, false,false, 0),
                        new Waypoints(0, 0, 0, false, false, true, false,false, 0),

                };
                executeWaypoints(wp3);

                //movimentação pelo meio
            }
            break;
        }

    }
    void executeWaypoints(Waypoints[] waypoints)
    {
        double speed = 0.5;
        double turnSpeed = 0.4;
        fieldOriented.init(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Elevator arm = new Elevator(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Claw claw = new Claw(hardwareMap);
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
                updateTelemetryAuto();
            }
            fieldOriented.fieldOrientedDrive(0, 0, 0);

            if(w.y<0)
                fieldOriented.fieldOrientedDrive(-speed,0, 0);
            else
                fieldOriented.fieldOrientedDrive(speed,0, 0);

            while(Math.abs(fieldOriented.getParallelPosition())<Math.abs( w.y)) {
                updateTelemetryAuto();
            }
            fieldOriented.fieldOrientedDrive(0, 0, 0);

            if(w.heading<0)
                fieldOriented.fieldOrientedDrive(0,0,-turnSpeed);
            else
                fieldOriented.fieldOrientedDrive(0,0,turnSpeed);

            while(Math.abs(fieldOriented.getRawExternalHeading())<Math.abs( w.heading)) {
                updateTelemetryAuto();
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
                updateTelemetryAuto();
            }
        }


    }
    void updateTelemetryAuto(){
        telemetry.addData("Heading",fieldOriented.getRawExternalHeading());
        telemetry.addData("Parallel",fieldOriented.getParallelPosition());
        telemetry.addData("Perpendicular",fieldOriented.getPerpendicularPosition());

        telemetry.update();
    }
    class Pipeline extends OpenCvPipeline{  //contrução do piperline
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        //função proscessFrame, irá detectar a localização geral de um objeto
        public Mat processFrame(Mat input){
            // e dividir o nosso espaço de vizualização em 3 quadrantes
            //parametros de altura e largura de cada quadrante
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_BGR2YCrCb); //converter o feedback da camera para YCbCr, deixa o desempenho melhor por otimiza-lo
            telemetry.addLine("Pipeline rodando");

            Rect leftRect = new Rect(1, 1, 639, 1079);
            Rect rightRect = new Rect(1280, 1, 639, 1079);
            Rect midRect = new Rect(640, 1, 639, 1079);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2); //esse código permite voce vizualizar os
            //quadrantes na sua driver hub, muito util para testes e verificação
            Imgproc.rectangle(output, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];  //linhas de escaneamento
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            telemetry.addData("Esquerda", leftavgfin);
            telemetry.addData("Meio", midavgfin);
            telemetry.addData("Direita", rightavgfin);
            telemetry.update();

            if (leftavgfin > rightavgfin && leftavgfin > midavgfin){ // linhas para printar em qual quadrante o
                //objeto está sendo identificado
                telemetry.addLine("Esquerda");
                linha = "Esquerda";
                direction = 2;

            } else if (rightavgfin > midavgfin){
                telemetry.addLine("Direita");
                linha = "Direita";
                //movimentação
                direction = 1;
            } else{
                telemetry.addLine("Meio");
                linha = "Meio";
                //movimentação:
                direction = 0;
            }

            return (output);
        }


    }


}