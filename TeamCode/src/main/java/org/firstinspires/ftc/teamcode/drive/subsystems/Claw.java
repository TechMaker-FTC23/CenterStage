package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private final double clawClosedPosition = 1.0;
    private final double clawOpenPosition = 0.5;

    private Servo claw;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");
    }

    public void activate(){
        claw.setPosition(clawOpenPosition);
    }
    public void reverse(){
        claw.setPosition(clawClosedPosition);
    }

}
