package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
        private DcMotor intakeMotor;

        public Intake(HardwareMap hmap){
            intakeMotor = hmap.dcMotor.get("intake");

            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        }



        public void activate(){intakeMotor.setPower(1);}

        public void close(){
            intakeMotor.setPower(-1);
        }

        public void stop(){
            intakeMotor.setPower(0);
        }



    }
