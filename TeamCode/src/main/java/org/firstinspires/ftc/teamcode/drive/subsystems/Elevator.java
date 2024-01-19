package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {

    public double armEncoder = 484.5;
    public double elevator_positionR = 0;
    public double elevator_positionL = 0;
    public double errorR = 0;
    public double errorL = 0;
    public double kP = 0.001;
    public double armVelocityR = 0;
    public double armVelocityL = 0;
    private DcMotor leftArm, rightArm;

    public Elevator(HardwareMap hmap) {
        leftArm = hmap.dcMotor.get("leftArm");
        rightArm = hmap.dcMotor.get("rightArm");

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public double getCurrentArmPosition() {
        double rightArmPosition = rightArm.getCurrentPosition();
        double leftArmPosition = leftArm.getCurrentPosition();
        return (rightArmPosition + leftArmPosition) / 2.0; //
    }
    public void activate() {
        elevator_positionL = armEncoder;
        elevator_positionR = armEncoder;
    }

    public void teste() {
        while(getCurrentArmPosition() < 100) {
            leftArm.setPower(0.7);
            rightArm.setPower(0.7);
        }
            leftArm.setPower(0);
            rightArm.setPower(0);
        }

    public void voltaTeste() {
        while (getCurrentArmPosition() > 0) {
            leftArm.setPower(-0.3);
            rightArm.setPower(-0.3);
        }
            leftArm.setPower(0);
            rightArm.setPower(0);
        }

    public void reverse() {
        elevator_positionR = 0;
        elevator_positionL = 0;
        leftArm.setTargetPosition((int)elevator_positionL);
        rightArm.setTargetPosition((int)elevator_positionR);
    }
    public void resetElevatorEncoders(){
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void task(){
        //calculo PID do braço;

            if(elevator_positionL==0 && elevator_positionR==0) {
                armVelocityL = -0.6;
                armVelocityR = armVelocityL;
                if(leftArm.getCurrentPosition()<10)
                    armVelocityL = 0;
                if(rightArm.getCurrentPosition()<10)
                    armVelocityR = 0;
            }else if(elevator_positionL==10 && elevator_positionR==10) {
                armVelocityL = -0.2;
                armVelocityR = armVelocityL;
                if (leftArm.getCurrentPosition() < 10)
                    armVelocityL = 0;
                if (rightArm.getCurrentPosition() < 10)
                    armVelocityR = 0;
                }else {
                errorL = elevator_positionL - leftArm.getCurrentPosition();
                errorR = elevator_positionR - rightArm.getCurrentPosition();
                armVelocityL = errorL * kP;
                armVelocityR = errorR * kP;

                leftArm.setPower(armVelocityL);
                rightArm.setPower(armVelocityR);
            }

        /*telemetry.addData("Atual", getCurrentArmPosition());
        telemetry.addData("Atual Esquerda", leftArm.getCurrentPosition());
        telemetry.addData("Atual Direita", rightArm.getCurrentPosition());
        telemetry.addData("Destino",arm_positionR);
        telemetry.addData("Erro Direita",errorR);
        telemetry.addData("erro esquerda",errorL);
        telemetry.addData("Velocidade",armVelocityR);
        telemetry.update();*/


    }
}


