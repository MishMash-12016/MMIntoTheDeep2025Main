package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class LinearIntakeEndUnitRotator extends SubsystemBase {

    private final static MMRobot robotInstance = MMRobot.getInstance();
    public static final double intakePose= 1;
    public static final double holdpose= 0.5;

    private final CuttleServo servo;


    public LinearIntakeEndUnitRotator(){
        servo = new CuttleServo(robotInstance.mmSystems.expansionHub, Configuration.END_UNIT_ROTATOR);

        servo.setPosition(0);
    }

    public Command setPosition(double newPos){
        return new InstantCommand(()-> {
            servo.setPosition(newPos);} ,
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