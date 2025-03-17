package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

@Config
public class LinearIntake extends SubsystemBase {

    private final MMRobot robotInstance = MMRobot.getInstance();

    private final Servo servoLeft;
    private final Servo servoRight;
    public double pose = 0;

    public static double config;

    public enum LinearIntakeState {
        MAX_OPENING(0.6),CLOSED_POSE(0);
        public double position;

        LinearIntakeState(double position){
            this.position = position;
        }
    }


    public LinearIntake(){
        servoLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L linear intake ");//3
        servoRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R linear intake ");//5
        servoRight.setPosition(1-LinearIntakeState.CLOSED_POSE.position);
        servoLeft.setPosition(LinearIntakeState.CLOSED_POSE.position);
        config =0.6;
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);
            pose = newPos;} ,
                this);
    }

    public Command setPosition(DoubleSupplier newPos){
        return new RunCommand(()-> {
            servoLeft.setPosition(newPos.getAsDouble());
            servoRight.setPosition(1-newPos.getAsDouble());
            pose = newPos.getAsDouble();} ,
                this);
    }

    public Command setPosition(LinearIntakeState state){
        return new InstantCommand(()-> {
            servoLeft.setPosition(state.position);
            servoRight.setPosition(1-state.position);
            pose = state.position;} ,
                this);
    }

    public void setPositionVoid(double newPos){
        servoLeft.setPosition(newPos);
        servoRight.setPosition(1-newPos);
        pose = newPos;
    }

    public void setPositionForConfig(){
        servoLeft.setPosition(config);
        servoRight.setPosition(config);
    }

    public Command defultCommand(double newPos){

        return new RunCommand(()-> {

            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);} ,
                this);
    }

    public Command setPositionByJoystick(DoubleSupplier doubleSupplier){
        return new RunCommand(()-> {
            double targetPose = Math.pow(doubleSupplier.getAsDouble(),3) * LinearIntakeState.MAX_OPENING.position;
            servoLeft.setPosition(targetPose);
            servoRight.setPosition(1 - targetPose);} ,
                this);
    }

    public double getPosition(){
        return servoLeft.getPosition();
    }

}