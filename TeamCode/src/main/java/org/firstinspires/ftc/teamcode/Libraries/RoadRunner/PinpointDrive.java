package org.firstinspires.ftc.teamcode.Libraries.RoadRunner;


import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

import java.util.function.DoubleSupplier;

/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
@Config
public class PinpointDrive extends MecanumDrive {
    public static class Params {
        /*
        Use the pinpoint IMU for tuning
        If true, overrides any IMU setting in MecanumDrive and uses exclusively Pinpoint for tuning
        You can also use the pinpoint directly in MecanumDrive if this doesn't work for some reason;
         replace "imu" with "pinpoint" or whatever your pinpoint is called in config.
         Note: Pinpoint IMU is always used for base localization
         */
        public boolean usePinpointIMUForTuning = true;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriverRR pinpoint;
    private Pose2d lastPinpointPose = pose;

    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("PINPOINT_PARAMS", PARAMS);
        pinpoint = MMSystems.localizer;


        if (PARAMS.usePinpointIMUForTuning) {
            lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(zyxOrientation(0, 0, 0)));
        }

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pose = new Pose2d(pose.position.x, pose.position.y, pose.heading.toDouble());

        pinpoint.setPosition(pose);

    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPinpointPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override the sensor's pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            pinpoint.setPosition(pose);
        }
        pinpoint.update();
        pose = pinpoint.getPositionRR();
        pose = new Pose2d(pose.position.x, pose.position.y, pose.heading.toDouble());
        lastPinpointPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE", new FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS", pinpoint.getDeviceStatus());

        return pinpoint.getVelocityRR();
    }


    // for debug logging
    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public FTCPoseMessage(Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x = pose.getX(DistanceUnit.INCH);
            this.y = pose.getY(DistanceUnit.INCH);
            this.heading = pose.getHeading(AngleUnit.RADIANS);
        }
    }

    public Command fieldOrientedDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier yaw) {
        return new RunCommand(
                () -> {
                    localizer.update();
                    pinpoint.update();
                    Vector2d joystickDirection = new Vector2d(x.getAsDouble(), y.getAsDouble());
                    Vector2d fieldOrientedVector = joystickDirection.rotateBy(Math.toDegrees(-pinpoint.getHeading()));
                    setPowerManually(fieldOrientedVector.getX(), fieldOrientedVector.getY(), yaw.getAsDouble());

                }, this
        ).whenFinished(()->setPowerManually(0,0,0));
    }

    public void resetRotation() {
        pinpoint.setPosition(new Pose2d(pinpoint.getPositionRR().component1(), 0));
    }

    public Command temp() {
        return new InstantCommand(() -> {
        }, this);
    }

    public boolean joystickMoved() {
        return Math.abs(MMRobot.getInstance().mmSystems.gamepadEx1.getLeftX()) > 0.05 ||
                Math.abs(MMRobot.getInstance().mmSystems.gamepadEx1.getRightX()) > 0.05 ||
                Math.abs(MMRobot.getInstance().mmSystems.gamepadEx1.getLeftY()) > 0.05 ||
                Math.abs(MMRobot.getInstance().mmSystems.gamepadEx1.getRightY()) > 0.05;
    }
}