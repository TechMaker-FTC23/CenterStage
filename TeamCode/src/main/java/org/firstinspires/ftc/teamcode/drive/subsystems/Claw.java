package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private final double clawOpenPosition = 0.8;
    private final double clawClosedPosition = 0.2;

    private final double clawInitialPosition = 0.5;
    private final double clawFinalPosition = 1.0;

    private Servo left;
    private Servo right;

    private Servo clawLeft;
    private Servo clawRight;



    public Claw(HardwareMap hardwareMap){
        clawRight = hardwareMap.servo.get("clawRight");
        clawLeft = hardwareMap.servo.get("clawLeft");
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
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

    public void initialPosition(){
        left.setPosition(clawInitialPosition);
        right.setPosition(1.0 - clawInitialPosition);
    }

    public void finalPosition(){
        left.setPosition(clawFinalPosition);
        right.setPosition(1.0 - clawFinalPosition);
    }
}
