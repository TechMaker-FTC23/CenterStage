package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Arm {

    public double armEncoder = 1000;
    public double climberPosition = 800;
    public double arm_positionR = 0;
    public double arm_positionL = 0;
    public double errorR = 0;
    public double errorL = 0;
    public double kP = 0.001;
    public double armVelocityR = 0;
    public double armVelocityL = 0;
    private DcMotor leftArm, rightArm;

    public Arm(HardwareMap hmap) {
        leftArm = hmap.dcMotor.get("leftArm");
        rightArm = hmap.dcMotor.get("rightArm");

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public double getCurrentArmPosition() {
        double rightArmPosition = rightArm.getCurrentPosition();
        double leftArmPosition = leftArm.getCurrentPosition();
        return (rightArmPosition + leftArmPosition) / 2.0; //
    }

    public void activate() {
        arm_positionL = armEncoder;
        arm_positionR = armEncoder;
    }

    public void climberPosition(){
        arm_positionL = climberPosition;
        arm_positionR = climberPosition;
        leftArm.setTargetPosition((int)arm_positionL);
        rightArm.setTargetPosition((int)arm_positionR);
    }

    public void reverse() {
        arm_positionL = 0;
        arm_positionR = 0;
        leftArm.setTargetPosition((int)arm_positionL);
        rightArm.setTargetPosition((int)arm_positionR);
    }
    public void resetArmEncoders(){
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void task(){
        //calculo PID do braço;

            if(arm_positionL==0) {
                armVelocityL = -0.6;
                armVelocityR = armVelocityL;
                if(leftArm.getCurrentPosition()<10)
                    armVelocityL = 0;
                if(rightArm.getCurrentPosition()<10)
                    armVelocityR = 0;
            }
            else {
                errorL = arm_positionL - leftArm.getCurrentPosition();
                errorR = arm_positionR - rightArm.getCurrentPosition();
                armVelocityL = errorL * kP;
                armVelocityR = errorR * kP;
            }
            leftArm.setPower(armVelocityL);
            rightArm.setPower(armVelocityR);

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


