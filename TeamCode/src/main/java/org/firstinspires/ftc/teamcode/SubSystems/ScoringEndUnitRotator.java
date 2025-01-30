
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

public class ScoringEndUnitRotator extends SubsystemBase {

    private final static MMRobot robotInstance = MMRobot.getInstance();
    public enum ScoringRotatorState {
        TRANSFER_POSE(0.61),
        SCORE_SAMPLE_POSE(0.15),
        MID_POSE_SPECIMEN(0.7),
        SCORE_SPECIMEN_POSE(0.44);

        public double position;
        ScoringRotatorState(double position){
            this.position = position;
        }}
    Servo servo;

    public ScoringEndUnitRotator(){
        servo = MMRobot.getInstance().mmSystems.hardwareMap.get(Servo.class, "Outake angle");
        servo.setPosition(ScoringRotatorState.TRANSFER_POSE.position);
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            servo.setPosition(newPos);} ,
                this);
    }
    public Command setPosition(ScoringRotatorState state){
        return new InstantCommand(()-> {
            servo.setPosition(state.position);} ,
                this);
    }

    public Command setPositionByJoystick(DoubleSupplier doubleSupplier){
        return new RunCommand(()-> {
            servo.setPosition(doubleSupplier.getAsDouble());} ,
                this);
    }

    public Double getTargetPosition(){
        return servo.getPosition();
    }
}

