package org.firstinspires.ftc.teamcode.drive;

public class Waypoints {
    public double x;
    public double y;
    public double right;
    public double left;
    public double heading;
    public boolean actIntake;
    public boolean actReverse;
    public boolean extendElevator;
    public boolean openClaw;
    public boolean closeClaw;
    public int timeout;
    public Waypoints(double x,double y, double heading,boolean actIntake,boolean actReverse,boolean extendElevator,boolean openClaw, boolean closeClaw,  int timeout){ //boolean actIntake, boolean actReverse,
    //boolean extendElevator, boolean openClaw,  int timeout){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.actIntake = actIntake;
        this.actReverse = actReverse;
        this.extendElevator = extendElevator;
        this.openClaw = openClaw;
        this.closeClaw = closeClaw;
        this.timeout = timeout;
    }
}
