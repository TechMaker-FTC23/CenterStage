package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    private DcMotor climberMotor;

    public Climber(HardwareMap hmap){
        climberMotor = hmap.dcMotor.get("climber");

        climberMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void activate(){
        climberMotor.setPower(1);
}
    public void reverse(){
        climberMotor.setPower(-1);
    }

    public void stop(){
        climberMotor.setPower(0);
    }


}
