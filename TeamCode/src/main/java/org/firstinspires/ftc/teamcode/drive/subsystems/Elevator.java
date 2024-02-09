package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator {

    public double armEncoder = 3003.5;
    public double elevator_positionR = 0;
    public double elevator_positionL = 0;
    public double errorR = 0;
    public double errorL = 0;
    public double kP = 0.010425781250000002;
    public double armVelocityR = 0;
    public double armVelocityL = 0;
    private DcMotor leftArm, rightArm;

    private final double clawInitialPosition = 0.5455;
    private final double clawFinalPosition = 0.36;

    private Servo leftServo;
    private Servo rightServo;

    public Elevator(HardwareMap hmap) {
        leftServo = hmap.servo.get("left");
        rightServo = hmap.servo.get("right");

        leftArm = hmap.dcMotor.get("leftElevator");
        rightArm = hmap.dcMotor.get("rightElevator");

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double right(){
        double rightPosition = rightArm.getCurrentPosition();
        return rightPosition;
    }

    public double left(){
        double leftPosition = leftArm.getCurrentPosition();
        return leftPosition;
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

    public void clawPosition(){
        if(getCurrentArmPosition()>1600){
            leftServo.setPosition(clawFinalPosition);
            rightServo.setPosition(1.0 - clawFinalPosition);
            }else{
            leftServo.setPosition(clawInitialPosition);
            rightServo.setPosition(1.0 - clawInitialPosition);
            }
        }


    public boolean finalPosition() {
        boolean ret = true;
        if (leftArm.getCurrentPosition() < armEncoder) {
            leftArm.setPower(0.7);
            ret = false;
        } else {
            leftArm.setPower(0);
        }
        if (rightArm.getCurrentPosition()<armEncoder) {
            rightArm.setPower(0.7);
            ret = false;
        } else {
            rightArm.setPower(0);
        }

        return ret;
    }

    public boolean initialPosition() {
        boolean ret = true;
        if (leftArm.getCurrentPosition() >0) {
            leftArm.setPower(-0.5);
            ret = false;
        } else {
            leftArm.setPower(0);
        }
        if (rightArm.getCurrentPosition()>0) {
            rightArm.setPower(-0.5);
            ret = false;
        } else {
            rightArm.setPower(0);
        }

        return ret;
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

            //pedro esteve aqui
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


