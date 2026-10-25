package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
controls are:
Left stick to control forward/backward as well as strafe
Right stick to control turn
B to push artifact into launcher
X to pull artifact out of launcher
A to fire artifact
*/
@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends LinearOpMode {
    // initialize motor names
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    //private DcMotor launcherWheel;
    private DcMotor ballPusher;
    private CRServo ballLoader;
    // private DcMotor intake;
    @Override
    public void runOpMode() {
        // initialize movement variables
        double turn;
        double forward;
        double strafe;
        double pow1;
        double pow2;
        boolean servoOn;
        // assign motors to motor names
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        // launcherWheel = hardwareMap.get(DcMotor.class, "LauncherWheel");
        //  intake = hardwareMap.get(DcMotor.class, "intake");
        ballLoader = hardwareMap.get(CRServo.class, "ballLoader");
        ballPusher = hardwareMap.get(DcMotor.class, "ballPusher");
        // set motor directions
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        // Drive robot
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // set movement variables to controller inputs
                forward = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x * 1.1;
                turn = gamepad1.right_stick_x;
                // set power caps
                pow1 = 1;
                pow2 = 1;
                // slow movement for precise artifact firing
                if (gamepad1.left_trigger > 0.1){
                    pow1 = 0.75 -(gamepad1.left_trigger / 2);
                }
                if (gamepad1.right_trigger > 0.1){
                    pow2 = 0.75 - (gamepad1.right_trigger/2);
                }
                if (gamepad2.right_trigger>0.3){
                    //    intake.setDirection(DcMotor.Direction.FORWARD);
                    ballPusher.setDirection(DcMotor.Direction.FORWARD);
                    //   launcherWheel.setDirection(DcMotor.Direction.REVERSE);
                    ballLoader.setDirection(DcMotor.Direction.REVERSE);
                }else{
                    //   intake.setDirection(DcMotor.Direction.REVERSE);
                    ballPusher.setDirection(DcMotor.Direction.REVERSE);
                    //    launcherWheel.setDirection(DcMotor.Direction.FORWARD);
                    ballLoader.setDirection(DcMotor.Direction.FORWARD);
                }
                if (gamepad1.a) {
                    //   launcherWheel(1);
                    //setDriveFTS(-1, 0, 0);
                    sleep(350);
                    //    intake(1);
                    // setDriveFTS(-1, 0, -1);//
                    sleep(55);//
                    sleep(230);

                }
                // control launcher and load artifacts into launcher
                //  if (gamepad2.a){
                //   launcherWheel.setPower(0.62);
                //}else{
                //     launcherWheel.setPower(0);
                //    }

                if (gamepad2.x){
                    ballPusher.setPower(1);
                }else{
                    ballPusher.setPower(0);
                }
                if (gamepad2.y){
                    ballLoader.setPower(1);

                }else{
                    ballLoader.setPower(0);
                }
//        if (gamepad2.b){
//          intake.setPower(1);
//        }else{
//          intake.setPower(0);
//        }
                frontLeft.setPower((forward*pow1)+(turn*pow2)+(strafe*pow1));
                backLeft.setPower((forward*pow1)+(turn*pow2)-(strafe*pow1));
                frontRight.setPower((forward*pow1)-(turn*pow2)-(strafe*pow1));
                backRight.setPower((forward*pow1)-(turn*pow2)+(strafe*pow1));
                // Put loop blocks here.
                telemetry.update();
            }
        } else {
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            //    launcherWheel.setPower(0);
            ballPusher.setPower(0);
        }
    }
}






