//package org.firstinspires.ftc.teamcode.utils;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.RotatedRect;
//
//public class Sample {
//    public Point lowest; // The lowest detected point of the sample in the image
//
//    public Point center;
//    private final Pose2d detectionPose; // The pose where the sample was detected
//    private double sampleX, sampleY, horizontalAngle, quality;
//    private double centerX, centerY;
//
//    private double angle;
//    public double orientation;
//    private Pose2d fieldPos; // Field-relative position of the sample
//    public double widthInches;
//    public double heightInches;
//
//    private double MAX_AREA=300000000;
//    private double MIN_AREA=0;
//
//
//    public Sample(Point lowest, Point center, RotatedRect rect, Pose2d detectionPose) {
//        this.lowest = lowest;
//        this.center = center;
//        this.detectionPose = detectionPose;
//        angle =  Math.toRadians(90 - rect.angle);
//    }
//
////    public Sample(Point lowest, Pose2d detectionPose, MatOfPoint contour) {
////        this.lowest = lowest;
////        this.detectionPose = detectionPose;
////        this.contour = contour;
////        calculatePosition();
////    }
//
//    // Getters for sample properties
//    public double getSampleX() {
//        return sampleX;
//    }
//    public double getCenterX() {
//        return centerX;
//    }
//
//    public double getSampleY() {
//        return sampleY;
//    }
//    public double getCenterY() {
//        return centerY;
//    }
//
//    public double getQuality() {
//        return quality;
//    }
//
//    public Pose2d getSamplePosition() {
//        return fieldPos;
//    }
//
//    public double getAngle(RotatedRect rect){
//        return angle;
//    }
//
//
//
//    // Calculates the sample position relative to the field
//    public void calculateField() {
//        double x = detectionPose.position.x + centerY * Math.cos(detectionPose.heading.toDouble()) - centerX * Math.sin(detectionPose.heading.toDouble());
//        double y = detectionPose.position.y + centerY * Math.sin(detectionPose.heading.toDouble()) + centerX * Math.cos(detectionPose.heading.toDouble());
//        fieldPos = new Pose2d(new Vector2d(x, y), orientation);
//    }
//
//    public boolean isTooBig() {
//        return (widthInches * heightInches >= MAX_AREA);
//    }
//
//    public boolean isTooSmall() {
//        return (widthInches * heightInches <=MIN_AREA);
//    }
//}