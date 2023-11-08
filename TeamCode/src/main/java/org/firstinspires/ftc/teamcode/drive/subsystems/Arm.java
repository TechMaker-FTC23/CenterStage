package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Arm {

    public double armEncoder = 900;
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

    public void reverse() {
        arm_positionL = 0;
        arm_positionR = 0;
    }
    public void resetArmEncoders(){
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void task(Telemetry telemetry){
        //calculo PID do braço;
            //errorL = arm_positionL - leftArm.getCurrentPosition();
            //errorR = arm_positionR - rightArm.getCurrentPosition();
            leftArm.setTargetPosition((int)arm_positionL);
            rightArm.setTargetPosition((int)arm_positionR);
            //armVelocityL = errorL * kP;
            //armVelocityR = errorR * kP;
            if(arm_positionL==0)
                armVelocityL = -0.3;
            else
                armVelocityL = 0.7;
            leftArm.setPower(armVelocityL);
            rightArm.setPower(armVelocityL);

        telemetry.addData("Atual", getCurrentArmPosition());
        telemetry.addData("Atual Esquerda", leftArm.getCurrentPosition());
        telemetry.addData("Atual Direita", rightArm.getCurrentPosition());
        telemetry.addData("Destino",arm_positionR);
        telemetry.addData("Erro Direita",errorR);
        telemetry.addData("erro esquerda",errorL);
        telemetry.addData("Velocidade",armVelocityR);
        telemetry.update();


    }
}


