package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

public class Elevator {


    private double activationHeight = 3300;
    private double armEncoder = 3490;
    private double armEncoderLeft = 3569;
    private double armEncoderRight = 3412;
    private double elevator_positionR = 0;
    private double elevator_positionL = 0;
    private double errorR = 0;
    private double errorL = 0;
    private double kP = 0.0025;
    private double kI = 0.0;
    private double kD = 0.00001;

    double integralL = 0; // Integral acumulada para o braço esquerdo
    double integralR = 0; // Integral acumulada para o braço direito

    double prevErrorL = 0;
    double prevErrorR = 0;
    private double armVelocityR = 0;
    private double armVelocityL = 0;
    private DcMotor leftArm, rightArm;
    private double lastTimestamp = 0;


    private Servo leftServo;
    private Servo rightServo;


    public Elevator(HardwareMap hmap) {

        leftArm = hmap.dcMotor.get("leftElevator");
        rightArm = hmap.dcMotor.get("rightElevator");

        leftArm.setDirection(DcMotor.Direction.REVERSE);
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


    /*
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
     */

    public double getCurrentArmPosition() {
        double rightArmPosition = rightArm.getCurrentPosition();
        double leftArmPosition = leftArm.getCurrentPosition();
        return (rightArmPosition + leftArmPosition) / 2.0; //
    }

    public void activate(){
        elevator_positionR = armEncoderRight;
        elevator_positionL = armEncoderLeft;
        leftArm.setTargetPosition((int)elevator_positionL);
        rightArm.setTargetPosition((int)elevator_positionR);
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


    public void task(double timestamp){
                double currentHeightL = calcularAlturaBraço(leftArm.getCurrentPosition());
                double currentHeightR = calcularAlturaBraço(rightArm.getCurrentPosition());

                errorL = elevator_positionL - leftArm.getCurrentPosition();
                errorR = elevator_positionR - rightArm.getCurrentPosition();
                double dt = timestamp - lastTimestamp;
                lastTimestamp = timestamp;
                double derivativeL = (errorL - prevErrorL)/dt;
                double derivativeR = (errorR - prevErrorR)/dt;

                if (currentHeightL > activationHeight) {
                    integralL += errorL * dt;
                }
                if (currentHeightR > activationHeight) {
                    integralR += errorR * dt;
                }

                armVelocityL = (errorL * kP) + (integralL * kI) + (kD * derivativeL);
                armVelocityR = (errorR * kP) + (integralR * kI) + (kD * derivativeR);


                prevErrorL = errorL;
                prevErrorR = errorR;

                if(elevator_positionL==0 && leftArm.getCurrentPosition()<10){
                    armVelocityL = 0;
                 }
                if(elevator_positionR==0 && rightArm.getCurrentPosition()<10){
                    armVelocityR = 0;
                }

                leftArm.setPower(armVelocityL);
                rightArm.setPower(armVelocityR);
            }
    private double calcularAlturaBraço(int encoderValue) {
        return encoderValue * 0.1;
    }

            //pedro esteve aqui, é melhor jogar que beijar



    }


