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
    public final double maxopening = 0.27;
    public final double offset = 0.25;
    public static final double transferPose = 0.21;


    public LinearIntake(){
        servoLeft = new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.LEFT_LINEAR_INTAKE);
        servoRight = new CuttleServo(robotInstance.mmSystems.controlHub, Configuration.RIGHT_LINEAR_INTAKE);

    }

    public Command setPosition(double newPos){

        return new InstantCommand(()-> {

            double setPosevar = newPos * maxopening  + offset;
            servoLeft.setPosition(setPosevar);
            servoRight.setPosition(1-setPosevar);} ,
                this);
    }

    public Command setPositionByJoystick(DoubleSupplier doubleSupplier){
        return new RunCommand(()-> {
            double targetPose = doubleSupplier.getAsDouble() * maxopening;
            targetPose += offset;
            servoLeft.setPosition(targetPose);
            servoRight.setPosition(1 - targetPose);} ,
                this);
    }

    public String getPosition(){
        return String.valueOf(servoLeft.getPosition())+" "+String.valueOf(servoRight.getPosition()) ;
    }
}