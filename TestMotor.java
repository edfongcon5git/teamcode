/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="Util LocateMotor 1", group="Mtr")
//@Disabled
public class TestMotor extends OpMode {

    DcMotor front_left, front_right;
    DcMotor back_left, back_right;

    float right_power, left_power;
	ElapsedTime MotorElapsedTime = new ElapsedTime();  // Time into current state

	int OperationState;
	int ToggleDirection;
	float current_position_value;


	public TestMotor() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */

        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");

		back_left = hardwareMap.dcMotor.get("back_left");
		back_right = hardwareMap.dcMotor.get("back_right");



		MotorElapsedTime.reset();
		OperationState = 0;
		ToggleDirection = 0 ;  // 0 == forward, 1=reverse

	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		if ( gamepad1.start){
			if ( ToggleDirection == 1 ) {
				ToggleDirection = 0;
			}
			else
			{
				ToggleDirection = 1;
			}
		}

		if (   ( gamepad1.y || gamepad1.b ||gamepad1.x ||gamepad1.a || gamepad1.right_bumper  )
			   && ( OperationState != 1 )
		   )
		{
			OperationState = 1;
			MotorElapsedTime.reset();

			if ( ToggleDirection == 1){
				front_left.setDirection(DcMotor.Direction.REVERSE);
				front_right.setDirection(DcMotor.Direction.REVERSE);
				back_left.setDirection(DcMotor.Direction.REVERSE);
				back_right.setDirection(DcMotor.Direction.REVERSE);
			}
			else
			{
				front_left.setDirection(DcMotor.Direction.FORWARD);
				front_right.setDirection(DcMotor.Direction.FORWARD);
				back_left.setDirection(DcMotor.Direction.FORWARD);
				back_right.setDirection(DcMotor.Direction.FORWARD);

			}

			if (gamepad1.y)
			{
				current_position_value = 0;
				front_left.setPower(.8);
				current_position_value = front_left.getCurrentPosition();
			}
			if (gamepad1.b) {
				current_position_value = 0;
				front_right.setPower(.8);
				current_position_value = front_right.getCurrentPosition();
			}
			if (gamepad1.x) {
				current_position_value = 0;
				back_left.setPower(.8);
				current_position_value = back_left.getCurrentPosition();
			}
			if (gamepad1.a) {
				current_position_value = 0;
				back_right.setPower(.8);
				current_position_value = back_right.getCurrentPosition();
			}
			if (gamepad1.right_bumper) {
				front_left.setPower(0);
				front_right.setPower(0);
				back_left.setPower(0);
				back_right.setPower(0);
			}
		}
	    else if ( OperationState == 1 ) {
			// Wait for Operation state 1 to reach the end of the time
			if (MotorElapsedTime.milliseconds() > 100) {
				OperationState = 0;
			}
		}
		else
		{
			left_power = -gamepad1.left_stick_y;
			right_power = -gamepad1.right_stick_y;

			right_power = Range.clip(right_power, -1, 1);
			left_power = Range.clip(left_power, -1, 1);

			right_power = (float) scaleInput(right_power);
			left_power = (float) scaleInput(left_power);

			back_right.setPower(right_power);
			front_right.setPower(right_power);

			back_left.setPower(left_power);
			front_left.setPower(left_power);
		}

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left power", String.format("%.2f", left_power));
        telemetry.addData("right power", String.format("%.2f", right_power));
		telemetry.addData("currentpos",String.format("%.1f", current_position_value));

		telemetry.addData("Time ", String.format("%4.1f ", MotorElapsedTime.time()) );
		telemetry.addData("Direction ", ToggleDirection );

		telemetry.addData("back right",back_right.getCurrentPosition());
		telemetry.addData("back left",back_left.getCurrentPosition());
		telemetry.addData("front right",front_right.getCurrentPosition());
		telemetry.addData("front left",front_left.getCurrentPosition());


	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

		float right_power = 0;
		float left_power = 0;

        back_right.setPower(right_power);
        front_right.setPower(right_power);

        back_left.setPower(left_power);
        front_left.setPower(left_power);
	}


	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
