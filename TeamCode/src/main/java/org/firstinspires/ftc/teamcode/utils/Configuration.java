package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * this file should represent ur configurations & ports.<p>
 * there are some examples here, ur welcome to change it however u want.</p>
 * please remember that cuttlefish is an experimental library,
 * if u dont get along with it, please go back to normal strings configuration.
 * <p>
 * notice that there are some things that i still haven't figured out how to use on cuttle fish,
 * for example: i still couldn't figure out how to use CR Servos.
 * in this case, i've used the normal string configuration to control it.
 * if u have more things that are missing, u can still combine the 2 methods.
 * im still not sure whether it just doesn't exist, or i don't know how to use it, so feel free to look around their docs and src.
 */
public class Configuration {
    public static final String IMU= "imu";
    public static final String DS1 = "DS1";

    //CONTROL HUB
    //DriveTrain
    public static final int DRIVE_TRAIN_BACK_LEFT = 1;
    public static final int DRIVE_TRAIN_FRONT_LEFT = 2;
    public static final int DRIVE_TRAIN_BACK_RIGHT = 0;
    public static final int DRIVE_TRAIN_FRONT_RIGHT = 3;

    public static final String AUTO_DRIVE_TRAIN_FRONT_LEFT = "leftFront"; //2
    public static final String AUTO_DRIVE_TRAIN_BACK_LEFT = "leftBack"; //1
    public static final String AUTO_DRIVE_TRAIN_FRONT_RIGHT = "rightFront"; //3
    public static final String AUTO_DRIVE_TRAIN_BACK_RIGHT = "rightBack"; //0


    public static final String PERPENDICULAR = AUTO_DRIVE_TRAIN_BACK_LEFT; // 1
    public static final String PARALLEL = AUTO_DRIVE_TRAIN_BACK_RIGHT; //0 //todo: choose the right motor, preferable port 0,3


    //CONTROL HUB
    //Linear Intake
    public static final int LEFT_LINEAR_INTAKE = 2;

    public static final int RIGHT_LINEAR_INTAKE = 3;

    //Linear Intake End Unit Rotator:
    public static final int LINEAR_END_UNIT_ROTATOR = 4;

    public static final int CLAW_INTAKE_SERVO = 0;

    //Linear Intake Arm:
    public static final int INTAKE_ARM_SERVO_RIGHT = 5;
    public static final int INTAKE_ARM_SERVO_LEFT = 1;




    //EXPANSION HUB
    public static final int SCORING_CLAW_SERVO = 2;
    public static final int SCORING_ROTATOR_SERVO = 3;
    //Scoring arm:
    public static final int SERVO_RIGHT_SCORING_ARM = 5;

    public static final int SERVO_LEFT_SCORING_ARM =4;



    //Elevator
    public static final int ELEVATOR1 = 0;
    public static final int ELEVATOR2 = 1;
    public static final int ELEVATOR3 = 2;

    public static final int ELEVATOR_ENCODER = 0;



}