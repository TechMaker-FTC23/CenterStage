package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private final double initPosition = 0.5;
    private final double clawOpenPosition = 0.5;
    private final double clawClosedPosition = 0.74;

    private Servo claw;



    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");

    }


    public void InitPosition(){
        claw.setPosition(initPosition);
    }

    public void close(){
        claw.setPosition(clawClosedPosition);
    }

    public void open(){

        claw.setPosition(clawOpenPosition);
    }

}
