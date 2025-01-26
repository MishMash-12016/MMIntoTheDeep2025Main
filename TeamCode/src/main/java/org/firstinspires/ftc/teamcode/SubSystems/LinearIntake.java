package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class LinearIntake extends SubsystemBase {

    private final MMRobot robotInstance = MMRobot.getInstance();

    private final Servo servoLeft;
    private final Servo servoRight;
    public static final double maxOpening = 0.6;
    public enum LinearIntakeState {
        OFFSET(0.22),CLOSED_POSE(0);
        public double position;

        LinearIntakeState(double position){
            this.position = position;
        }
    }


    public LinearIntake(){
        servoLeft = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "L linear intake ");
        servoRight = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "R linear intake ");
        servoRight.setPosition(1);
        servoLeft.setPosition(0);

    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);} ,
                this);
    }
    public Command setPosition(DoubleSupplier newPos){
        return new RunCommand(()-> {
            servoLeft.setPosition(newPos.getAsDouble());
            servoRight.setPosition(1-newPos.getAsDouble());} ,
                this);
    }
    public Command setPosition(LinearIntakeState state){
        return new InstantCommand(()-> {
            servoLeft.setPosition(state.position);
            servoRight.setPosition(1-state.position);} ,
                this);
    }


    public Command defultCommand(double newPos){

        return new RunCommand(()-> {

            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);} ,
                this);
    }

    public Command setPositionByJoystick(DoubleSupplier doubleSupplier){
        return new RunCommand(()-> {
            double targetPose = Math.pow(doubleSupplier.getAsDouble(),3) * maxOpening;
            servoLeft.setPosition(targetPose);
            servoRight.setPosition(1 - targetPose);} ,
                this);
    }

    public String getPosition(){
        return String.valueOf(servoLeft.getPosition())+" "+String.valueOf(servoRight.getPosition()) ;
    }
}