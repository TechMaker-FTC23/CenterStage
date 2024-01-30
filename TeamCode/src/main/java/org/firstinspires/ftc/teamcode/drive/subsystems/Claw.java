package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private final double clawOpenPosition = 0.8;
    private final double clawClosedPosition = 0.2;

    private Servo clawLeft;
    private Servo clawRight;



    public Claw(HardwareMap hardwareMap){
        clawRight = hardwareMap.servo.get("clawRight");
        clawLeft = hardwareMap.servo.get("clawLeft");
    }

    public void openRight(){
        clawRight.setPosition(1.0 - clawOpenPosition);
    }

    public void openLeft(){
        clawLeft.setPosition(clawOpenPosition);
    }

    public void close(){
        clawLeft.setPosition(clawClosedPosition);
        clawRight.setPosition(1.0 - clawClosedPosition);
    }

    public void open(){
        clawLeft.setPosition(clawOpenPosition);
        clawRight.setPosition(1.0 - clawOpenPosition);
    }

}
