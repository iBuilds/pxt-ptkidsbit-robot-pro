/**
 * Functions are mapped to blocks using various macros
 * in comments starting with %. The most important macro
 * is "block", and it specifies that a block should be
 * generated for an **exported** function.
 */

let inputString = ""
let adc_value = 0
let position_value = 0
let button_state = 0

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

enum Sensor {
    //% block="Front"
    Front,
    //% block="Back"
    Back,
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum LED_Color {
    //% block="Red"
    Red = 1,
    //% block="Green"
    Green = 2,
    //% block="Blue"
    Blue = 3,
    //% block="White"
    White = 4,
    //% block="Black"
    Black = 5,
}

enum LED_FB {
    //% block="Front"
    Front,
    //% block="Back"
    Back
}

enum LED_LR {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Sensor_Status {
    //% block="ON"
    ON = 1,
    //% block="OFF"
    OFF = 0
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
            adc_value = parseFloat(inputString.split(",")[2])
        }
        else if (inputString.split(",")[0] == "RP") {
            position_value = parseFloat(inputString.split(",")[2])
        }
        else if (inputString.split(",")[0] == "RB") {
            button_state = parseFloat(inputString.split(",")[2])
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
        serial.writeLine(data)
    }

    //% group="Motor Control"
    /**
     * Stop all Motor
     */
    //% block="Motor Stop"
    export function motorStop(): void {
        sendDataSerial("MC,0,0")
        basic.pause(10)
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
        sendDataSerial("CP," + left + "," + right)
        basic.pause(10)
        serial.redirectToUSB()
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
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (spin == _Spin.Right) {
            sendDataSerial("MC," + speed + "," + -speed)
            basic.pause(10)
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
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (turn == _Turn.Right) {
            sendDataSerial("MC," + speed + ",0")
            basic.pause(10)
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
        basic.pause(10)
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
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (motor == Motor_Write.Motor_Right) {
            sendDataSerial("MC,0," + speed)
            basic.pause(10)
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
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (servo == Servo_Write.S1) {
            sendDataSerial("SC,1," + degree)
            basic.pause(10)
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
        basic.pause(10)
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
        basic.pause(10)
        serial.redirectToUSB()
        return button_state
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
        return adc_value
    }

    //% group="Sensor and ADC"
    /**
     * Read Distance from Ultrasonic Sensor
     */
    //% block="GETDistance"
    export function distanceRead(maxCmDistance = 500): number {
        return 0
    }

    //% group="Sensor and ADC"
    /**
     * Turn on and turn off Line Sensor 
     */
    //% block="Sensor %sensor|Power $sensor_status"
    export function sensorPower(sensor: Sensor, sensor_status: Sensor_Status): void {
        let _sensor = ""
        if (sensor == Sensor.Front) _sensor = "f"
        else if (sensor == Sensor.Back) _sensor = "b"
        else if (sensor == Sensor.Left) _sensor = "l"
        else if (sensor == Sensor.Right) _sensor = "r"
        sendDataSerial("PL," + _sensor + "," + sensor_status)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Turn on and turn off Line Sensor All
     */
    //% block="Sensor Power $sensor_status"
    export function sensorPowerAll(sensor_status: Sensor_Status): void {
        sendDataSerial("PA," + sensor_status)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Color Left and Right
     */
    //% block="Set Color %LED_LR|Sensor 1  $color_1|Sensor 2  $color_2|Sensor 3  $color_3|Sensor 4  $color_4|Sensor 5  $color_5"
    export function setColorLR(sensor: LED_LR, color_1: LED_Color, color_2: LED_Color, color_3: LED_Color, color_4: LED_Color, color_5: LED_Color): void {
        let _sensor = ""
        if (sensor == LED_LR.Left) _sensor = "l"
        else if (sensor == LED_LR.Right) _sensor = "r"
        sendDataSerial("SL," + _sensor + "," + color_1 + "," + color_2 + "," + color_3 + "," + color_4 + "," + color_5)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Color Front and Black
     */
    //% block="Set Color %LED_FB|Sensor 1  $color_1|Sensor 2  $color_2|Sensor 3  $color_3|Sensor 4  $color_4|Sensor 5  $color_5|Sensor 6  $color_6|Sensor 7  $color_7"
    export function setColorFB(sensor: LED_FB, color_1: LED_Color, color_2: LED_Color, color_3: LED_Color, color_4: LED_Color, color_5: LED_Color, color_6: LED_Color, color_7: LED_Color): void {
        let _sensor = ""
        if (sensor == LED_FB.Front) _sensor = "f"
        else if (sensor == LED_FB.Back) _sensor = "b"
        sendDataSerial("SL," + _sensor + "," + color_1 + "," + color_2 + "," + color_3 + "," + color_4 + "," + color_5 + "," + color_6 + "," + color_7)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Color All
     */
    //% block="Set Color $color"
    export function setColor(color: LED_Color): void {
        sendDataSerial("SA," + color)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
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
        
    }

    //% group="Line Follower"
    /**
     * Read Position Line
     */
    //% block="Position Read $sensor"
    export function positionRead(sensor: Sensor): number {
        let _sensor = ""
        if (sensor == Sensor.Front) _sensor = "f"
        else if (sensor == Sensor.Back) _sensor = "b"
        else if (sensor == Sensor.Left) _sensor = "l"
        else if (sensor == Sensor.Right) _sensor = "r"
        sendDataSerial("RP," + _sensor)
        basic.pause(10)
        serial.redirectToUSB()
        return position_value
    }

    //% group="Line Follower"
    /**
     * Calibrate Sensor
     */
    //% block="Set Calibrate Sensor $mode"
    export function SensorCalibrate(mode: Caribrate_Mode): void {
        if (mode == Caribrate_Mode.MODE1) {
            sendDataSerial("CS,1")
        }
        else if (mode == Caribrate_Mode.MODE2) {
            sendDataSerial("CS,2")
        }
        inputString = serial.readLine()
        basic.pause(10)
        serial.redirectToUSB()
    }
}