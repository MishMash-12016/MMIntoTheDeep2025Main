package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

import java.util.function.DoubleSupplier;

public class LinearIntake extends SubsystemBase {

    private final MMRobot robotInstance = MMRobot.getInstance();

    private final CuttleServo servoLeft;
    private final CuttleServo servoRight;
    public final double maxOpening = 0.6;
    public final double offset = 0.22;
    public static final double closedPose = -0.1;


    public LinearIntake(){
        servoLeft = new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.LEFT_LINEAR_INTAKE);
        servoRight = new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.RIGHT_LINEAR_INTAKE);

    }

    public Command setPosition(double newPos){

        return new InstantCommand(()-> {

            servoLeft.setPosition(newPos);
            servoRight.setPosition(1-newPos);} ,
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