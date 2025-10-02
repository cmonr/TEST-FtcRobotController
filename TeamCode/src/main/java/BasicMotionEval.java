/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Motion Eval", group="Idk What This Is For")
public class BasicMotionEval extends LinearOpMode {

    static final double ROBOT_UPDATE_RATE_MS = 1000.0/10;
    static final double TELEMETRY_UPDATE_RATE_MS = 1000.0/4;
    static final double SERVO_MAX_POS = 1.00;
    static final double SERVO_MIN_POS = 0.00;

    private final ElapsedTime gameTimer = new ElapsedTime();
    private final ElapsedTime robotTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();


    // Motors
    private DcMotorEx[] motors;

    // Servos
    private Servo[] servos;


    // Track power levels
    private double[] motorPowers = null;
    private double[] servoPowers = null;

    // Servos
    private int activeServoNdx;


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that strings mentioned here are set within
        // the robot's configuration, and must exist within the "active" configuration.
        motors = new DcMotorEx[]{
            hardwareMap.get(DcMotorEx.class, "m0"),
            hardwareMap.get(DcMotorEx.class, "m1"),
            hardwareMap.get(DcMotorEx.class, "m2"),
            hardwareMap.get(DcMotorEx.class, "m3")
        };

        servos = new Servo[]{
            hardwareMap.get(Servo.class, "s0"),
            hardwareMap.get(Servo.class, "s1"),
            hardwareMap.get(Servo.class, "s2"),
            hardwareMap.get(Servo.class, "s3"),
            hardwareMap.get(Servo.class, "s4"),
            hardwareMap.get(Servo.class, "s5")
        };


        // Track power levels
        motorPowers = new double[]{ 0.0, 0.0, 0.0, 0.0 };
        servoPowers = new double[]{ 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };

        // Init controllable servos
        activeServoNdx = 0;

        // Init motors and servos
        for(int ndx=0; ndx < motors.length; ndx++) { motors[ndx].setPower(motorPowers[ndx]); }
        for(int ndx=0; ndx < servos.length; ndx++) { servos[ndx].setPosition(servoPowers[ndx]); }


        // Wait until driver presses PLAY button
        telemetry.addData("Status", "READY");
        telemetry.update();
        waitForStart();

        // Run until driver presses STOP button
        gameTimer.reset();
        robotTimer.reset();
        telemetryTimer.reset();
        while (opModeIsActive()) {
            if(robotTimer.milliseconds() >= ROBOT_UPDATE_RATE_MS ) {
                robotTimer.reset();

                // Set motor powers
                motorPowers[0] = gamepad1.left_stick_x;
                motorPowers[1] = gamepad1.left_stick_y * -1;
                motorPowers[2] = gamepad1.right_stick_x;
                motorPowers[3] = gamepad1.right_stick_y * -1;
                for (int ndx = 0; ndx < motors.length; ndx++) {
                    motors[ndx].setPower(motorPowers[ndx]);
                }


                // Update active servos
                if (gamepad1.dpadRightWasReleased()) {
                    activeServoNdx++;
                    if (activeServoNdx >= servos.length) {
                        activeServoNdx = 0;
                    }
                }
                if (gamepad1.dpadLeftWasReleased()) {
                    activeServoNdx--;
                    if (activeServoNdx <= 0) {
                        activeServoNdx = servos.length - 1;
                    }
                }

                // Update active servo powers
                if (gamepad1.dpadUpWasReleased()) {
                    servoPowers[activeServoNdx] += 0.05;
                } else if (gamepad1.dpadDownWasReleased()) {
                    servoPowers[activeServoNdx] -= 0.05;
                }

                if(servoPowers[activeServoNdx] > SERVO_MAX_POS ) {
                    servoPowers[activeServoNdx] = SERVO_MAX_POS;
                }
                if(servoPowers[activeServoNdx] < SERVO_MIN_POS ) {
                    servoPowers[activeServoNdx] = SERVO_MIN_POS;
                }

                // Set servo power
                //  Use "A" and "B" buttons as overrides for servos.
                //  Quick and easy way to set to max and set to min, while not forgetting the
                //   servo's last position.
                servos[activeServoNdx].setPosition(
                    (gamepad1.b) ? SERVO_MAX_POS :
                        ((gamepad1.a) ? SERVO_MIN_POS :
                            servoPowers[activeServoNdx]));

            }

            if(telemetryTimer.milliseconds() >= TELEMETRY_UPDATE_RATE_MS ) {
                telemetryTimer.reset();

                telemetry.addData("Elapsed Time", gameTimer.toString());
                // Show data to driver
                telemetry.addData("Motors", "%.02f, %.02f, %.02f, %.02f",
                        motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
                telemetry.addData("Servos", "%.02f, %.02f, %.02f, %.02f, %.02f, %.02f",
                    servos[0].getPosition(),
                    servos[1].getPosition(),
                    servos[2].getPosition(),
                    servos[3].getPosition(),
                    servos[4].getPosition(),
                    servos[5].getPosition()
                );
                telemetry.addData("Active Servo", "%d", activeServoNdx);
                telemetry.update();
            }
        }
    }
}
