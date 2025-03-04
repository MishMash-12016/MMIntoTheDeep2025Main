package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RedFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

         double maxWheelVel = 70;
        double maxProfileAccel = 70;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder (new Pose2d(5.5, -65.5, Math.toRadians(90)))
                        .setTangent(Math.toRadians(270))
                        .lineToLinearHeading(new Pose2d(50, -59.2,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-2, -25, Math.toRadians(90)))

                        //Second specimen
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                        .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-2, -25, Math.toRadians(90)))

                        //Third specimen
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                        .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-5, -25, Math.toRadians(90)))

                        //Forth specimen
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                        .lineToLinearHeading(new Pose2d(45, -59.2, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-5, -25, Math.toRadians(90)))

                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(90)), Math.atan((-42.0 + 57) / (12.0 - 45)))
                        .lineToLinearHeading(new Pose2d(45, -57, Math.toRadians(90)))

//
////--
//
//                            //first
                        .setTangent(Math.toRadians(270))
                        .lineToLinearHeading(new Pose2d(50, -58.2, Math.toRadians(90)))




                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


