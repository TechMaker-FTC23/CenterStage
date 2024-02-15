package org.firstinspires.ftc.teamcode.drive;

public class Waypoints {
    public double foward;
    public double back;
    public double right;
    public double left;
    public double heading;
    //public boolean actIntake;
    //public boolean actReverse;
    //public boolean extendElevator;
    //public boolean openClaw;
    public int timeout;
    public Waypoints(double foward,double back,double right, double left, double heading, int timeout){ //boolean actIntake, boolean actReverse,
    //boolean extendElevator, boolean openClaw,  int timeout){
        this.foward = foward;
        this.back = back;
        this.right = right;
        this.left = left;
        this.heading = heading;
        //this.actIntake = actIntake;
        //this.actReverse = actReverse;
        //this.extendElevator = extendElevator;
        //this.openClaw = openClaw;
        this.timeout = timeout;
    }
}
