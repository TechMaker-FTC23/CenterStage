package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Led {
    private DcMotor luzes;

    public Led(HardwareMap hmap){
        luzes = hmap.dcMotor.get("led");
    }

    public void ligaLed(){
        luzes.setPower(0.5);
    }

    public void desligaLed(){
        luzes.setPower(0);
    }



}
