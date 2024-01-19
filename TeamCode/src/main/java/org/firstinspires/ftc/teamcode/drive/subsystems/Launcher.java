package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

        private Servo laucherLeft;
        private Servo launcherRight;

        private final double launchPosition = 0.5;
        private final double closed = 1;

        public Launcher(HardwareMap hardwareMap){
            laucherLeft = hardwareMap.servo.get("launcherLeft");
            launcherRight = hardwareMap.servo.get("launcherRight");
        }

        public void activate(){
            laucherLeft.setPosition(launchPosition);
            launcherRight.setPosition(1.0 - launchPosition);
        }

        public void stop(){
            laucherLeft.setPosition(closed);
            launcherRight.setPosition(1.0 - closed);
        }
    }

