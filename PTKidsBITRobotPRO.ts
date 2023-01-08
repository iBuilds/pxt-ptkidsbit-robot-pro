/**
 * Functions are mapped to blocks using various macros
 * in comments starting with %. The most important macro
 * is "block", and it specifies that a block should be
 * generated for an **exported** function.
 */

let inputString = ""
let adc_value: number[] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
let button_state: number[] = [0, 0, 0]



let Sensor_All_PIN = [6, 5, 4, 3, 2, 1, 0, 8, 14, 13, 12, 11, 10, 9]
let Sensor_PIN_Front = [5, 4, 3, 2, 1]
let Sensor_Left_Front = [6]
let Sensor_Right_Front = [0]
let Sensor_PIN_Back = [13, 12, 11, 10, 9]
let Sensor_Left_Back = [14]
let Sensor_Right_Back = [8]
let Sensor_Center_Left = 7
let Sensor_Center_Right = 15
let Num_Sensor = 5
let KP = 0.004
let KD = 0.04
let Max_Speed = 100

let Version = 1
let Read_Version = false
let IMU_Address = 0x68
let PCA = 0x40
let initI2C = false
let initLED = false
let SERVOS = 0x06
let Line_LOW_Front = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_HIGH_Front = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_LOW_Center = [0, 0]
let Line_HIGH_Center = [0, 0]
let Color_Line_All_Front: number[] = []
let Color_Background_All_Front: number[] = []
let Color_Line_Front: number[] = []
let Color_Background_Front: number[] = []
let Color_Line_Left_Front: number[] = []
let Color_Background_Left_Front: number[] = []
let Color_Line_Center_Left: number[] = []
let Color_Background_Center_Left: number[] = []
let Color_Line_Right_Front: number[] = []
let Color_Background_Right_Front: number[] = []
let Color_Line_Center_Right: number[] = []
let Color_Background_Center_Right: number[] = []
let Line_All_Front = [0, 0, 0, 0, 0, 0, 0]
let Line_LOW_Back = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_HIGH_Back = [0, 0, 0, 0, 0, 0, 0, 0]
let Color_Line_All_Back: number[] = []
let Color_Background_All_Back: number[] = []
let Color_Line_Back: number[] = []
let Color_Background_Back: number[] = []
let Color_Line_Left_Back: number[] = []
let Color_Background_Left_Back: number[] = []
let Color_Line_Right_Back: number[] = []
let Color_Background_Right_Back: number[] = []
let Line_All_Back = [0, 0, 0, 0, 0, 0, 0]
let Line_Mode = 0
let Last_Position = 0
let Compensate_Left = 0
let Compensate_Right = 0
let error = 0
let P = 0
let D = 0
let previous_error = 0
let PD_Value = 0
let left_motor_speed = 0
let right_motor_speed = 0
let Servo_8_Enable = 0
let Servo_12_Enable = 0
let Servo_8_Degree = 0
let Servo_12_Degree = 0
let distance = 0
let timer = 0

enum Motor_Write {
    //% block="Left"
    Motor_Left,
    //% block="Right"
    Motor_Right
}

enum _Turn {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum _Spin {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Servo_Write {
    //% block="S0"
    S0,
    //% block="S1"
    S1
}

enum Button_Name {
    //% block="BUTTONA"
    BUTTONA = 0,
    //% block="BUTTONB"
    BUTTONB = 1,
    //% block="BUTTONC"
    BUTTONC = 2
}

enum Button_Status {
    //% block="Pressed"
    Pressed = 1,
    //% block="Released"
    Released = 0
}

enum ADC_Read {
    //% block="ADC0"
    ADC0 = 0,
    //% block="ADC1"
    ADC1 = 1,
    //% block="ADC2"
    ADC2 = 2,
    //% block="ADC3"
    ADC3 = 3,
    //% block="ADC4"
    ADC4 = 4,
    //% block="ADC5"
    ADC5 = 5,
    //% block="ADC6"
    ADC6 = 6,
    //% block="ADC7"
    ADC7 = 7,
    //% block="ADC8"
    ADC8 = 8,
    //% block="ADC9"
    ADC9 = 9,
    //% block="ADC10"
    ADC10 = 10,
    //% block="ADC11"
    ADC11 = 11,
    //% block="ADC12"
    ADC12 = 12,
    //% block="ADC13"
    ADC13 = 13,
    //% block="ADC14"
    ADC14 = 14,
    //% block="ADC15"
    ADC15 = 15,
    //% block="ADC16"
    ADC16 = 16,
    //% block="ADC17"
    ADC17 = 17,
    //% block="ADC18"
    ADC18 = 18,
    //% block="ADC19"
    ADC19 = 19,
    //% block="ADC20"
    ADC20 = 20,
    //% block="ADC21"
    ADC21 = 21,
    //% block="ADC22"
    ADC22 = 22,
    //% block="ADC23"
    ADC23 = 23
}

enum Caribrate_Mode {
    //% block="MODE1"
    MODE1,
    //% block="MODE2"
    MODE2
}

enum Forward_Direction {
    //% block="Forward"
    Forward,
    //% block="Backward"
    Backward
}

enum Intersection {
    //% block="|\n\u2513\n|"
    Left,
    //% block="|\n\u2533\n|"
    Center,
    //% block="|\n\u250F\n|"
    Right
}

enum Turn_Line {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Turn_Sensor {
    //% block="Center"
    Center,
    //% block="ADC1"
    ADC1,
    //% block="ADC2"
    ADC2,
    //% block="ADC3"
    ADC3,
    //% block="ADC4"
    ADC4
}

enum Direction_Robot {
    //% block="Front"
    Front,
    //% block="Back"
    Back
}

enum Stop_Position {
    //% block="Front"
    Front,
    //% block="Center"
    Center,
    //% block="Back"
    Back
}


//% color="#48C9B0" icon="\u2707"
namespace PTKidsBITRobotPRO {
    serial.onDataReceived(serial.delimiters(Delimiters.NewLine), function () {
        inputString = serial.readString()
        inputString = inputString.substr(0, inputString.length - 2)
        if (inputString.split(",")[0] == "RS") {
            adc_value[parseFloat(inputString.split(",")[1])] = parseFloat(inputString.split(",")[2])
        }
        else if (inputString.split(",")[0] == "RB") {
            button_state[parseFloat(inputString.split(",")[1])] = parseFloat(inputString.split(",")[2])
        }
    })

    function sendDataSerial(data: string) {
        inputString = ""
        serial.setWriteLinePadding(0)
        serial.redirect(
            SerialPin.P1,
            SerialPin.P8,
            BaudRate.BaudRate115200
        )
        basic.pause(10)
        serial.writeLine(data)
    }

    //% group="Motor Control"
    /**
     * Stop all Motor
     */
    //% block="Motor Stop"
    export function motorStop(): void {
        sendDataSerial("MC,0,0")
        serial.redirectToUSB()
    }

    //% group="Motor Control"
    /**
     * Compensate Speed Motor Left and Motor Right
     */
    //% block="Compensate Left %Motor_Left|Compensate Right %Motor_Right"
    //% left.min=-100 left.max=100
    //% right.min=-100 right.max=100
    export function motorCompensate(left: number, right: number): void {
        Compensate_Left = left
        Compensate_Right = right
    }

    //% group="Motor Control"
    /**
     * Spin the Robot to Left or Right. The speed motor is adjustable between 0 to 100.
     */
    //% block="Spin %_Spin|Speed %Speed"
    //% speed.min=0 speed.max=255
    export function Spin(spin: _Spin, speed: number): void {
        if (spin == _Spin.Left) {
            sendDataSerial("MC," + -speed + "," + speed)
            serial.redirectToUSB()
        }
        else if (spin == _Spin.Right) {
            sendDataSerial("MC," + speed + "," + -speed)
            serial.redirectToUSB()
        }
    }

    //% group="Motor Control"
    /**
     * Turn the Robot to Left or Right. The speed motor is adjustable between 0 to 100.
     */
    //% block="Turn %_Turn|Speed %Speed"
    //% speed.min=0 speed.max=255
    export function Turn(turn: _Turn, speed: number): void {
        if (turn == _Turn.Left) {
            sendDataSerial("MC,0," + speed)
            serial.redirectToUSB()
        }
        else if (turn == _Turn.Right) {
            sendDataSerial("MC," + speed + ",0")
            serial.redirectToUSB()
        }
    }

    //% group="Motor Control"
    /**
     * Control motors speed both at the same time. The speed motors is adjustable between -100 to 100.
     */
    //% block="Motor Left %Motor_Left|Motor Right %Motor_Right"
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    export function motorGo(speed1: number, speed2: number): void {
        sendDataSerial("MC," + speed1 + "," + speed2)
        serial.redirectToUSB()
    }

    //% group="Motor Control"
    /**
     * Control motor speed 1 channel. The speed motor is adjustable between -100 to 100.
     */
    //% block="Motor Write %Motor_Write|Speed %Speed"
    //% speed.min=-255 speed.max=255
    export function motorWrite(motor: Motor_Write, speed: number): void {
        if (motor == Motor_Write.Motor_Left) {
            sendDataSerial("MC," + speed + ",0")
            serial.redirectToUSB()
        }
        else if (motor == Motor_Write.Motor_Right) {
            sendDataSerial("MC,0," + speed)
            serial.redirectToUSB()
        }
    }

    //% group="Servo Control"
    /**
     * Control Servo Motor 0 - 180 Degrees
     */
    //% block="Servo %Servo_Write|Degree %Degree"
    //% degree.min=0 degree.max=180
    export function servoWrite(servo: Servo_Write, degree: number): void {
        if (servo == Servo_Write.S0) {
            sendDataSerial("SC,0," + degree)
            serial.redirectToUSB()
        }
        else if (servo == Servo_Write.S1) {
            sendDataSerial("SC,1," + degree)
            serial.redirectToUSB()
        }
    }

    //% group="Sound"
    /**
     * Play Sound on Buzzer
     */
    //% block="Tone $note|Duration %duration"
    //% note.min=0 note.max=2000
    //% duration.min=0 duration.max=5000
    //% note.defl=800
    //% duration.defl=100
    export function tone(note: number, duration: number): void {
        sendDataSerial("PS," + note + "," + duration)
        serial.redirectToUSB()
    }

    //% group="Sensor and ADC"
    /**
     * Runs the program when the button is pressed
     */
    //% block="When $button|is %duration"
    export function onButton(button: Button_Name, status: Button_Status): void {
        while (PTKidsBITRobotPRO.buttonRead(button) != status);
    }

    //% group="Sensor and ADC"
    /**
     * Read Button Status
     */
    //% block="Button Read $button"
    export function buttonRead(button: Button_Name): number {
        sendDataSerial("RB," + button)
        basic.pause(200)
        serial.redirectToUSB()
        return button_state[button]
    }

    //% group="Sensor and ADC"
    /**
     * Read Analog from ADC Channel
     */
    //% block="ADC Read $pin"
    export function ADCRead(pin: ADC_Read): number {
        sendDataSerial("RS," + pin)
        basic.pause(10)
        serial.redirectToUSB()
        return adc_value[pin]
    }

    //% group="Sensor and ADC"
    /**
     * Read Distance from Ultrasonic Sensor
     */
    //% block="GETDistance"
    export function distanceRead(maxCmDistance = 500): number {
        let duration

        if (control.millis() - timer > 1000) {
            pins.setPull(DigitalPin.P1, PinPullMode.PullNone)
            pins.digitalWritePin(DigitalPin.P1, 0)
            control.waitMicros(2)
            pins.digitalWritePin(DigitalPin.P1, 1)
            control.waitMicros(10)
            pins.digitalWritePin(DigitalPin.P1, 0)
            duration = pins.pulseIn(DigitalPin.P2, PulseValue.High, maxCmDistance * 58)
            distance = Math.idiv(duration, 58)
        }

        pins.setPull(DigitalPin.P1, PinPullMode.PullNone)
        pins.digitalWritePin(DigitalPin.P1, 0)
        control.waitMicros(2)
        pins.digitalWritePin(DigitalPin.P1, 1)
        control.waitMicros(10)
        pins.digitalWritePin(DigitalPin.P1, 0)
        duration = pins.pulseIn(DigitalPin.P2, PulseValue.High, maxCmDistance * 58)
        let d = Math.idiv(duration, 58)

        if (d != 0) {
            distance = (0.1 * d) + (1 - 0.1) * distance
        }
        timer = control.millis()
        return Math.round(distance)
    }

    //% group="Line Follower"
    /**
     * Turn Left or Right Follower Line Mode
     */
    //% block="TurnLINE %turn|Speed\n %speed|Sensor %sensor|Fast Time %time"
    //% speed.min=0 speed.max=100
    //% time.shadow="timePicker"
    //% break_delay.shadow="timePicker"
    //% time.defl=200
    export function TurnLINE(turn: Turn_Line, speed: number, sensor: Turn_Sensor, time: number) {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        let on_line = 0
        let adc_sensor_pin = sensor - 1
        let error = 0
        let motor_speed = 0
        let motor_slow = 20
        let timer = control.millis()
        let _position = 0
        let _position_min = 0
        let _position_max = 0

        while (1) {
            on_line = 0
            for (let i = 0; i < Sensor_PIN_Front.length; i++) {
                if ((pins.map(ADCRead(ADC_PIN[Sensor_PIN_Front[i]]), Color_Line_All_Front[i], Color_Background_All_Front[i], 1000, 0)) >= 500) {
                    on_line += 1;
                }
            }

            if (on_line == 0) {
                break
            }

            error = timer - (control.millis() - time)
            motor_speed = error

            if (motor_speed > speed) {
                motor_speed = speed
            }
            else if (motor_speed < 0) {
                motor_speed = motor_slow
            }

            if (turn == Turn_Line.Left) {
                motorGo(-motor_speed, motor_speed)
            }
            else if (turn == Turn_Line.Right) {
                motorGo(motor_speed, -motor_speed)
            }
        }
        while (1) {
            // if ((pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[adc_sensor_pin]]), Color_Line[adc_sensor_pin], Color_Background[adc_sensor_pin], 1000, 0)) >= 800) {
            //     motorStop()
            //     break
            // }
            if (sensor == Turn_Sensor.Center) {
                _position_min = 1250
                _position_max = 1750
            }
            else if (sensor == Turn_Sensor.ADC1) {
                _position_min = 2500
                _position_max = 3000
            }
            else if (sensor == Turn_Sensor.ADC2) {
                _position_min = 1500
                _position_max = 2000
            }
            else if (sensor == Turn_Sensor.ADC3) {
                _position_min = 1000
                _position_max = 1500
            }
            else if (sensor == Turn_Sensor.ADC4) {
                _position_min = 0
                _position_max = 500
            }

            _position = GETPosition(Direction_Robot.Front)
            if (_position > _position_min && _position < _position_max) {
                motorStop()
                break
            }
            else {
                error = timer - (control.millis() - time)
                motor_speed = error

                if (motor_speed > speed) {
                    motor_speed = speed
                }
                else if (motor_speed < 0) {
                    motor_speed = motor_slow
                }

                if (turn == Turn_Line.Left) {
                    motorGo(-motor_speed, motor_speed)
                }
                else if (turn == Turn_Line.Right) {
                    motorGo(motor_speed, -motor_speed)
                }
            }
        }
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward Timer
     */
    //% block="Time %time|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    //% time.shadow="timePicker"
    //% time.defl=200
    export function ForwardTIME(time: number, min_speed: number, max_speed: number, kp: number, kd: number) {
        let timer = control.millis()

        while (control.millis() - timer < time) {
            error = GETPosition(Direction_Robot.Back) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed + PD_Value
            right_motor_speed = min_speed - PD_Value

            if (left_motor_speed > max_speed) {
                left_motor_speed = max_speed
            }
            else if (left_motor_speed < -max_speed) {
                left_motor_speed = -max_speed
            }

            if (right_motor_speed > max_speed) {
                right_motor_speed = max_speed
            }
            else if (right_motor_speed < -max_speed) {
                right_motor_speed = -max_speed
            }

            motorGo(-left_motor_speed, -right_motor_speed)
        }
        motorStop()
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Custom Line
     */
    //% block="Find %sensor|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% sensor.defl="0--111"
    //% min_speed.defl=30
    //% max_speed.defl=100
    //% kp.defl=0.01
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    export function ForwardLINECustom(sensor: string, min_speed: number, max_speed: number, kp: number, kd: number) {
        
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Line
     */
    //% block="Junction\n\n %Intersection|Direction\n %Forward_Direction|Count Line %count|Base Speed %base_speed|Stop Point %Stop_Position"
    //% find.defl=Intersection.Center
    //% count.defl=1
    //% min_speed.defl=30
    //% min_speed.min=0 min_speed.max=100
    //% stop_position.defl=Stop_Position.Center
    export function ForwardLINECount(find: Intersection, Direction: Forward_Direction, count: number, min_speed: number, stop_position: Stop_Position) {
        
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Line
     */
    //% block="Junction\n\n %Find_Line|Direction\n %Forward_Direction|Base Speed %base_speed|Stop Point %Stop_Position"
    //% find.defl=Intersection.Center
    //% min_speed.defl=30
    //% min_speed.min=0 min_speed.max=100
    //% stop_position.defl=Stop_Position.Center
    export function ForwardLINE(find: Intersection, Direction: Forward_Direction, min_speed: number, stop_position: Stop_Position) {
        
    }

    //% group="Line Follower"
    /**
     * Basic Line Follower
     */
    //% block="Direction %Forward_Direction|Speed %base_speed"
    //% min_speed.defl=30
    //% min_speed.min=0 min_speed.max=100
    export function Follower(Direction: Forward_Direction, min_speed: number) {
        if (Direction == Forward_Direction.Forward) {
            error = GETPosition(Direction_Robot.Front) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            if (min_speed == 0) {
                PD_Value = (0.005 * P) + (0.001 * D)
                previous_error = error
            }
            else {
                PD_Value = (KP * P) + (KD * D)
                previous_error = error
            }

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value

            if (left_motor_speed > Max_Speed) {
                left_motor_speed = Max_Speed
            }
            else if (left_motor_speed < -Max_Speed) {
                left_motor_speed = -Max_Speed
            }

            if (right_motor_speed > Max_Speed) {
                right_motor_speed = Max_Speed
            }
            else if (right_motor_speed < -Max_Speed) {
                right_motor_speed = -Max_Speed
            }

            motorGo(left_motor_speed, right_motor_speed)
        }
        else {
            error = GETPosition(Direction_Robot.Back) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (KP * P) + (KD * D)
            previous_error = error

            left_motor_speed = min_speed + PD_Value
            right_motor_speed = min_speed - PD_Value

            if (left_motor_speed > Max_Speed) {
                left_motor_speed = Max_Speed
            }
            else if (left_motor_speed < -Max_Speed) {
                left_motor_speed = -Max_Speed
            }

            if (right_motor_speed > Max_Speed) {
                right_motor_speed = Max_Speed
            }
            else if (right_motor_speed < -Max_Speed) {
                right_motor_speed = -Max_Speed
            }

            motorGo(-left_motor_speed, -right_motor_speed)
        }
    }

    //% group="Line Follower"
    /**
     * Get Position Line
     */
    //% block="GETPosition %Direction_Robot"
    export function GETPosition(Direction: Direction_Robot): number {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        let Average = 0
        let Sum_Value = 0
        let ON_Line = 0

        if (Direction == Direction_Robot.Front) {
            for (let i = 0; i < Num_Sensor; i++) {
                let Value_Sensor = 0;
                if (Line_Mode == 0) {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Front[i]]), Color_Line_Front[i], Color_Background_Front[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                else {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Front[i]]), Color_Background_Front[i], Color_Line_Front[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                if (Value_Sensor > 200) {
                    ON_Line = 1;
                }
                Average += Value_Sensor * (i * 1000)
                Sum_Value += Value_Sensor
            }
        }
        else {
            for (let i = 0; i < Num_Sensor; i++) {
                let Value_Sensor = 0;
                if (Line_Mode == 0) {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Back[i]]), Color_Line_Back[i], Color_Background_Back[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                else {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Back[i]]), Color_Background_Back[i], Color_Line_Back[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                if (Value_Sensor > 200) {
                    ON_Line = 1;
                }
                Average += Value_Sensor * (i * 1000)
                Sum_Value += Value_Sensor
            }
        }

        if (ON_Line == 0) {
            if (Last_Position < (Num_Sensor - 1) * 1000 / 2) {
                return (Num_Sensor - 1) * 1000
            }
            else {
                return 0
            }
        }
        Last_Position = Average / Sum_Value;
        return Math.round(((Num_Sensor - 1) * 1000) - Last_Position)
    }

    //% group="Line Follower"
    /**
     * Calibrate Sensor
     */
    //% block="Calibrate Sensor $mode"
    export function SensorCalibrate(mode: Caribrate_Mode): void {
        if (mode == Caribrate_Mode.MODE1) {
            sendDataSerial("CS,1")
        }
        else if (mode == Caribrate_Mode.MODE2) {
            sendDataSerial("CS,2")
        }
        inputString = serial.readLine()
        serial.redirectToUSB()
    }
}