
package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class ScoringEndUnitRotator extends SubsystemBase {

    private final static MMRobot robotInstance = MMRobot.getInstance();
    public enum ScoringRotatorState {
        HOLD_POSE(1),
        RIGHT_ANGLE(0.28),
        SCORE_SAMPLE_POSE(0);

        public double position;
        ScoringRotatorState(double position){
            this.position = position;
        }}

    private final CuttleServo servo;


    public ScoringEndUnitRotator(){
        servo = new CuttleServo(robotInstance.mmSystems.expansionHub, Configuration.SCORING_ROTATOR_SERVO);
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

