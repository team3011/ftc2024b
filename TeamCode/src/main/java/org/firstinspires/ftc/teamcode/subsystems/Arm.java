package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private JulliansClaw claw;
    private Lift lift;
    private Wrist wrist;

    public Arm(DcMotorEx sm, TouchSensor st, DcMotorEx tm, Servo lc, Servo rc, DcMotorEx lm, Servo lw, Servo rw){
        this.shoulder = new Shoulder(sm, st);
        this.telescope = new Telescope(tm);
        this.claw = new JulliansClaw(lc, rc);
        this.lift = new Lift(lm);
        this.wrist = new Wrist(lw, rw);
    }

    public void manualMove(double input){
        this.shoulder.moveManual(input);
        this.telescope.moveManual(input/3.33);
    }

    public void updateEverything(){
        this.telescope.update();
    }
    public int shouldEncoder(){
        return this.shoulder.getEncoderValue();
    }

    public int telescopeEncoder(){
        return this.telescope.getEncoderValue();
    }
}
