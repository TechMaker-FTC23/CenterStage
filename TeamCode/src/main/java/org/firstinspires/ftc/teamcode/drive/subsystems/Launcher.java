package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

        private Servo laucherLeft;

        private final double launchPosition = 0.5;
        private final double closed = 0.0;

        public Launcher(HardwareMap hardwareMap){
            laucherLeft = hardwareMap.servo.get("launcher");
        }

        public void activate(){
            laucherLeft.setPosition(launchPosition);
        }

        public void stop(){
            laucherLeft.setPosition(closed);
        }
    }

