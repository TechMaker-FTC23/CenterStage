package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

        private Servo laucher;

        private final double launchPosition = 0.5;
        private final double closed = 1;

        public Launcher(HardwareMap hardwareMap){
            laucher = hardwareMap.servo.get("launcher");
        }

        public void activate(){
            laucher.setPosition(launchPosition);
        }

        public void stop(){
            laucher.setPosition(closed);
        }
    }

