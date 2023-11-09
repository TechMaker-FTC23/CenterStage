package org.firstinspires.ftc.teamcode.drive;

public class Waypoints {
    public double x;
    public double y;
    public double heading;
    public boolean actIntake;
    public boolean actReverse;
    public boolean extendArm;
    public boolean openClaw;
    public int timeout;
    public Waypoints(double x,double y, double heading, boolean actIntake, boolean actReverse,
    boolean extendArm, boolean openClaw,  int timeout){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.actIntake = actIntake;
        this.actReverse = actReverse;
        this.extendArm = extendArm;
        this.openClaw = openClaw;
        this.timeout = timeout;
    }
}
